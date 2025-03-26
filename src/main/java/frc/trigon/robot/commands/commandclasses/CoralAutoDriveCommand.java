package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.flippable.FlippableRotation2d;

import java.util.function.Supplier;

public class CoralAutoDriveCommand extends ParallelCommandGroup {
    private static final double AUTO_COLLECTION_OPENING_CHECK_DISTANCE_METERS = 2.2;
    private static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0.3, 0, 0.03);
    private static final ProfiledPIDController X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
            new ProfiledPIDController(2.6, 0, 0, new TrapezoidProfile.Constraints(2.9, 5.7));
    private static final ObjectDetectionCamera CAMERA = CameraConstants.OBJECT_DETECTION_CAMERA;
    private Translation2d distanceFromTrackedCoral;

    public CoralAutoDriveCommand() {
        addCommands(
                new InstantCommand(CAMERA::initializeTracking),
                getTrackCoralCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        getDriveToCoralCommand(() -> distanceFromTrackedCoral),
                        GeneralCommands.getFieldRelativeDriveCommand(),
                        () -> CAMERA.getTrackedObjectFieldRelativePosition() != null
                )
        );
    }

    private Command getTrackCoralCommand() {
        return new RunCommand(() -> {
            CAMERA.trackObject(SimulatedGamePieceConstants.GamePieceType.CORAL);
            distanceFromTrackedCoral = calculateDistanceFromTrackedCoral();
        });
    }

    public static Translation2d calculateDistanceFromTrackedCoral() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d trackedObjectPositionOnField = CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectPositionOnField == null)
            return null;

        final Translation2d difference = robotPose.getTranslation().minus(trackedObjectPositionOnField);
        var a = difference.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("Distance", a);
        Logger.recordOutput("TargetXDistance", X_PID_CONTROLLER.getSetpoint().position);
        return a;
    }

    public static Command getDriveToCoralCommand(Supplier<Translation2d> distanceFromTrackedCoral) {
        return new SequentialCommandGroup(
//                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
//                        () -> 0,
//                        () -> Y_PID_CONTROLLER.calculate(distanceFromTrackedCoral.get().getY()),
//                        CoralAutoDriveCommand::calculateTargetAngle
//                ).until(() -> shouldMoveTowardsCoral(distanceFromTrackedCoral.get())),
                new InstantCommand(() -> X_PID_CONTROLLER.reset(distanceFromTrackedCoral.get().getX())),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> X_PID_CONTROLLER.calculate(distanceFromTrackedCoral.get().getX(), -0.65),
                        () -> Y_PID_CONTROLLER.calculate(distanceFromTrackedCoral.get().getY()),
                        CoralAutoDriveCommand::calculateTargetAngle
                )
        );
    }

    public static boolean shouldMoveTowardsCoral(Translation2d distanceFromTrackedCoral) {
        return distanceFromTrackedCoral != null &&
                (distanceFromTrackedCoral.getNorm() > AUTO_COLLECTION_OPENING_CHECK_DISTANCE_METERS ||
                        RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR));
    }

    public static FlippableRotation2d calculateTargetAngle() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d trackedObjectFieldRelativePosition = CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectFieldRelativePosition == null)
            return null;

        final Translation2d objectDistanceToRobot = trackedObjectFieldRelativePosition.minus(robotPose.getTranslation());
        final Rotation2d angularDistanceToObject = new Rotation2d(Math.atan2(objectDistanceToRobot.getY(), objectDistanceToRobot.getX()));
        return new FlippableRotation2d(angularDistanceToObject, false);
    }
}