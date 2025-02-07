package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
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
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.misc.leds.LEDCommands;
import org.trigon.hardware.misc.leds.LEDStrip;
import org.trigon.utilities.flippable.FlippableRotation2d;

public class CoralAlignmentCommand extends ParallelCommandGroup {
    public static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0.4, 0, 0.03);
    private static final ObjectDetectionCamera CAMERA = CameraConstants.OBJECT_DETECTION_CAMERA;

    public CoralAlignmentCommand() {
        addCommands(
                new InstantCommand(CAMERA::initializeTracking),
                getSetLEDColorsCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        getDriveWhileAligningToCoralCommand(),
                        GeneralCommands.getFieldRelativeDriveCommand(),
                        () -> CAMERA.getTrackedObjectFieldRelativePosition() != null
                ),
                getTrackCoralCommand()
        );
    }

    private Command getTrackCoralCommand() {
        return new RunCommand(() -> CAMERA.trackObject(SimulatedGamePieceConstants.GamePieceType.CORAL));
    }

    private Command getSetLEDColorsCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDCommands.getStaticColorCommand(Color.kGreen, LEDStrip.LED_STRIPS),
                LEDCommands.getStaticColorCommand(Color.kRed, LEDStrip.LED_STRIPS),
                () -> CAMERA.hasTargets(SimulatedGamePieceConstants.GamePieceType.CORAL)
        );
    }

    private Command getDriveWhileAligningToCoralCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                this::calculateSwerveYPowerOutput,
                this::calculateTargetAngle
        );
    }

    private double fieldRelativePowersToSelfRelativeXPower(double xPower, double yPower) {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();
        final double xValue = CommandConstants.calculateDriveStickAxisValue(xPower);
        final double yValue = CommandConstants.calculateDriveStickAxisValue(yPower);

        return (xValue * robotHeading.getCos()) + (yValue * robotHeading.getSin());
    }

    private double calculateSwerveYPowerOutput() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d trackedObjectPositionOnField = CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectPositionOnField == null)
            return 0;

        final Translation2d difference = robotPose.getTranslation().minus(trackedObjectPositionOnField);
        final Translation2d selfRelativeDifference = difference.rotateBy(robotPose.getRotation().unaryMinus());
        return Y_PID_CONTROLLER.calculate(selfRelativeDifference.getY());
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