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
            new PIDController(0.6, 0, 0);
    private static final ObjectDetectionCamera CAMERA = CameraConstants.OBJECT_DETECTION_CAMERA;
    private Translation2d targetCoralTranslation = null;

    public CoralAlignmentCommand() {
        addCommands(
                new InstantCommand(CAMERA::initializeTracking),
                new InstantCommand(() -> targetCoralTranslation = null),
                getSetLEDColorsCommand(),
                GeneralCommands.getContinuousConditionalCommand(
                        getDriveWhileAligningToCoralCommand(),
                        GeneralCommands.duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND),
                        () -> targetCoralTranslation != null
                ),
                getTrackCoralCommand()
        );
    }

    private Command getTrackCoralCommand() {
        return new RunCommand(() -> {
            CAMERA.trackObject(SimulatedGamePieceConstants.GamePieceType.CORAL);
            if (CAMERA.getTrackedObjectPositionOnField() != null && targetCoralTranslation == null)
                targetCoralTranslation = CAMERA.getTrackedObjectPositionOnField();
        });
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
                this::calculateYControllerOutput,
                this::calculateTargetAngle
        );
    }

    private double fieldRelativePowersToSelfRelativeXPower(double xPower, double yPower) {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();
        final double xValue = CommandConstants.calculateDriveStickAxisValue(xPower);
        final double yValue = CommandConstants.calculateDriveStickAxisValue(yPower);

        return (xValue * robotHeading.getCos()) + (yValue * robotHeading.getSin());
    }

    private double calculateYControllerOutput() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d difference = robotPose.getTranslation().minus(targetCoralTranslation);
        final Translation2d selfRelativeDifference = difference.rotateBy(robotPose.getRotation().unaryMinus());
        return Y_PID_CONTROLLER.calculate(selfRelativeDifference.getY());
    }

    private FlippableRotation2d calculateTargetAngle() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d difference = targetCoralTranslation.minus(robotPose.getTranslation());
        final Rotation2d differenceAngle = new Rotation2d(Math.atan2(difference.getY(), difference.getX()));
        return new FlippableRotation2d(differenceAngle, false);
    }
}