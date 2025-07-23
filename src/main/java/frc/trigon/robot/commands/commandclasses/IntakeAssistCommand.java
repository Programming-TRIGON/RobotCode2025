package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

import java.util.function.Supplier;

public class IntakeAssistCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.OBJECT_DETECTION_CAMERA;
    private static final ProfiledPIDController
            X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
            new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5)),
            Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
                    new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
                    new ProfiledPIDController(0.3, 0, 0.03, new TrapezoidProfile.Constraints(2.65, 5.5)),
            THETA_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
                    new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
                    new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5));
    private Translation2d distanceFromTrackedCoral;

    public IntakeAssistCommand(AssistMode assistMode) {
        addCommands(
                new InstantCommand(CAMERA::initializeTracking),
                getTrackCoralCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        GeneralCommands.getFieldRelativeDriveCommand(),
                        getAssistIntakeCommand(assistMode, () -> distanceFromTrackedCoral),
                        () -> CAMERA.getTrackedObjectFieldRelativePosition() == null
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
        Translation2d robotToTrackedCoralDistance = difference.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("IntakeAssist/TrackedCoralDistance", robotToTrackedCoralDistance);
        return robotToTrackedCoralDistance;
    }

    public static Command getAssistIntakeCommand(AssistMode assistMode, Supplier<Translation2d> distanceFromTrackedCoral) {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> OperatorConstants.DEFAULT_INTAKE_ASSIST_MODE.shouldAssistX ? getXAssistedPower(assistMode, distanceFromTrackedCoral.get()) : getXJoystickPower(),
                () -> OperatorConstants.DEFAULT_INTAKE_ASSIST_MODE.shouldAssistY ? getYAssistedPower(assistMode, distanceFromTrackedCoral.get()) : getYJoystickPower(),
                () -> OperatorConstants.DEFAULT_INTAKE_ASSIST_MODE.shouldAssistTheta ? getThetaAssistedPower(assistMode, distanceFromTrackedCoral.get()) : OperatorConstants.DRIVER_CONTROLLER.getRightX()
        );
    }

    private static double getXAssistedPower(AssistMode assistMode, Translation2d distanceFromTrackedCoral) {
        return calculateTranslationAssistPower(assistMode, distanceFromTrackedCoral.getX(), X_PID_CONTROLLER, getXJoystickPower());
    }

    private static double getYAssistedPower(AssistMode assistMode, Translation2d distanceFromTrackedCoral) {
        return calculateTranslationAssistPower(assistMode, distanceFromTrackedCoral.getY(), Y_PID_CONTROLLER, getYJoystickPower());
    }

    private static double getThetaAssistedPower(AssistMode assistMode, Translation2d distanceFromTrackedCoral) {
        return calculateRotationAssistPower(assistMode, distanceFromTrackedCoral.getAngle().plus(Rotation2d.k180deg));
    }

    private static double getXJoystickPower() {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();

        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftX();
        final double
                xValue = CommandConstants.calculateDriveStickAxisValue(joystickX),
                yValue = CommandConstants.calculateDriveStickAxisValue(joystickY);

        return (xValue * robotHeading.getCos()) + (yValue * robotHeading.getSin());
    }

    private static double getYJoystickPower() {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();

        final double
                joystickX = OperatorConstants.DRIVER_CONTROLLER.getLeftY(),
                joystickY = OperatorConstants.DRIVER_CONTROLLER.getLeftX();
        final double
                xValue = CommandConstants.calculateDriveStickAxisValue(joystickX),
                yValue = CommandConstants.calculateDriveStickAxisValue(joystickY);

        return (yValue * robotHeading.getCos()) - (xValue * robotHeading.getSin());
    }

    private static double calculateTranslationAssistPower(AssistMode assistMode, double distance, ProfiledPIDController pidController, double joystickValue) {
        final double pidOutput = pidController.calculate(distance);

        if (assistMode.equals(AssistMode.ALTERNATE_ASSIST))
            return calculateAlternateAssistPower(pidOutput, joystickValue);

        final double
                assistPower = pidOutput * OperatorConstants.INTAKE_ASSIST_SCALAR,
                stickPower = joystickValue * (1 - OperatorConstants.INTAKE_ASSIST_SCALAR);
        return assistPower + stickPower;
    }

    private static double calculateRotationAssistPower(AssistMode assistMode, Rotation2d angleOffset) {
        final double
                pidOutput = THETA_PID_CONTROLLER.calculate(angleOffset.getRadians()),
                joystickValue = OperatorConstants.DRIVER_CONTROLLER.getRightX();

        if (assistMode.equals(AssistMode.ALTERNATE_ASSIST))
            return calculateAlternateAssistPower(pidOutput, joystickValue);

        final double
                assistPower = pidOutput * OperatorConstants.INTAKE_ASSIST_SCALAR,
                stickPower = joystickValue * (1 - OperatorConstants.INTAKE_ASSIST_SCALAR);
        return assistPower + stickPower;
    }

    private static double calculateAlternateAssistPower(double pidOutput, double joystickValue) {
        return pidOutput * (1 - Math.abs(joystickValue)) * Math.signum(joystickValue) + joystickValue;
    }

    public enum AssistMode {
        ALTERNATE_ASSIST(true, true, true),
        FULL_ASSIST(true, true, true),
        ALIGN_ASSIST(false, true, true),
        ASSIST_ROTATION(false, false, true);

        final boolean
                shouldAssistX,
                shouldAssistY,
                shouldAssistTheta;

        AssistMode(boolean shouldAssistX, boolean shouldAssistY, boolean shouldAssistTheta) {
            this.shouldAssistX = shouldAssistX;
            this.shouldAssistY = shouldAssistY;
            this.shouldAssistTheta = shouldAssistTheta;
        }
    }
}