package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.misc.leds.LEDCommands;
import org.trigon.hardware.misc.leds.LEDStrip;
import org.trigon.utilities.flippable.FlippableRotation2d;

public class CoralAutoDriveCommand extends ParallelCommandGroup {
    private static final PIDController
            X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0, 0, 0),
            Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
                    new PIDController(0.5, 0, 0) :
                    new PIDController(0.4, 0, 0.03);
    private static final ObjectDetectionCamera CAMERA = CameraConstants.OBJECT_DETECTION_CAMERA;
    private Translation2d distanceFromTrackedCoral;

    public CoralAutoDriveCommand() {
        addCommands(
                new InstantCommand(CAMERA::initializeTracking),
                getSetLEDColorsCommand(),
                getTrackCoralCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        getDriveToCoralWhenReadyToIntakeCommand(),
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

    private Command getSetLEDColorsCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDCommands.getStaticColorCommand(Color.kGreen, LEDStrip.LED_STRIPS),
                LEDCommands.getStaticColorCommand(Color.kRed, LEDStrip.LED_STRIPS),
                () -> CAMERA.getTrackedObjectFieldRelativePosition() != null
        );
    }

    private Command getDriveToCoralWhenReadyToIntakeCommand() {
        return new SequentialCommandGroup(
                getDriveToCoralCommand().until(() -> distanceFromTrackedCoral.getNorm() < CoralIntakeConstants.AUTO_COLLECTION_OPENING_CHECK_DISTANCE_METERS),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, () -> 0).until(() -> RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR)),
                getDriveToCoralCommand()
        );
    }

    private Command getDriveToCoralCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> X_PID_CONTROLLER.calculate(distanceFromTrackedCoral.getX()),
                () -> Y_PID_CONTROLLER.calculate(distanceFromTrackedCoral.getY()),
                this::calculateTargetAngle
        );
    }

    private Translation2d calculateDistanceFromTrackedCoral() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d trackedObjectPositionOnField = CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectPositionOnField == null)
            return new Translation2d();

        final Translation2d difference = robotPose.getTranslation().minus(trackedObjectPositionOnField);
        return difference.rotateBy(robotPose.getRotation().unaryMinus());
    }

    private FlippableRotation2d calculateTargetAngle() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d trackedObjectFieldRelativePosition = CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectFieldRelativePosition == null)
            return null;

        final Translation2d objectDistanceToRobot = trackedObjectFieldRelativePosition.minus(robotPose.getTranslation());
        final Rotation2d angularDistanceToObject = new Rotation2d(Math.atan2(objectDistanceToRobot.getY(), objectDistanceToRobot.getX()));
        return new FlippableRotation2d(angularDistanceToObject, false);
    }
}