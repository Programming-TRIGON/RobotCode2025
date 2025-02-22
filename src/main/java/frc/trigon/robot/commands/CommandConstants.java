package frc.trigon.robot.commands;

import com.ctre.phoenix.led.LarsonAnimation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.CoralCollectionCommands;
import frc.trigon.robot.commands.commandfactories.CoralPlacingCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.commands.CameraPositionCalculationCommand;
import org.trigon.commands.WheelRadiusCharacterizationCommand;
import org.trigon.hardware.misc.XboxController;
import org.trigon.hardware.misc.leds.LEDCommands;
import org.trigon.hardware.misc.leds.LEDStrip;
import org.trigon.utilities.flippable.FlippableRotation2d;

/**
 * A class that contains commands that only use parameters and don't require logic.
 * These are different from {@link GeneralCommands} and command factories because they don't contain any logic, and only run an existing command with parameters.
 * For example, the static LED command always changes all the LED strips to the same LED mode with the same settings.
 */
public class CommandConstants {
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;
    private static final double
            MINIMUM_TRANSLATION_SHIFT_POWER = 0.18,
            MINIMUM_ROTATION_SHIFT_POWER = 0.3;
    private static final double JOYSTICK_ORIENTED_ROTATION_DEADBAND = 0.07;

    public static final Command
            FIELD_RELATIVE_DRIVE_WITH_JOYSTICK_ORIENTED_ROTATION_TO_REEF_SECTIONS_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
            CommandConstants::calculateJoystickOrientedToReefSectionsTargetAngle
    ),
            SELF_RELATIVE_DRIVE_COMMAND = SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
                    () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
            ),
    //            RESET_HEADING_COMMAND = new InstantCommand(RobotContainer.POSE_ESTIMATOR::resetHeading),
    RESET_HEADING_COMMAND = new InstantCommand(() -> RobotContainer.POSE_ESTIMATOR.resetPose(RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose())),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                    () -> getXPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> getYPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> 0
            ),
            STATIC_WHITE_LED_COLOR_COMMAND = LEDCommands.getStaticColorCommand(Color.kWhite, LEDStrip.LED_STRIPS),
            DEFAULT_LEDS_COMMAND = LEDCommands.getBreatheCommand(Color.kLightSeaGreen, LEDConstants.DEFAULT_COMMAND_BREATHING_LEDS_AMOUNT, LEDConstants.DEFAULT_COMMAND_BREATHING_SPEED, LEDConstants.DEFAULT_COMMAND_BREATHING_IS_INVERTED, LarsonAnimation.BounceMode.Front, LEDStrip.LED_STRIPS),
            WHEEL_RADIUS_CHARACTERIZATION_COMMAND = new WheelRadiusCharacterizationCommand(
                    PathPlannerConstants.ROBOT_CONFIG.moduleLocations,
                    RobotContainer.SWERVE::getDriveWheelPositionsRadians,
                    () -> RobotContainer.SWERVE.getHeading().getRadians(),
                    (omegaRadiansPerSecond) -> RobotContainer.SWERVE.selfRelativeDriveWithoutSetpointGeneration(new ChassisSpeeds(0, 0, omegaRadiansPerSecond), null),
                    RobotContainer.SWERVE
            ),
            OVERRIDE_IS_CLIMBING_COMMAND = new InstantCommand(() -> RobotContainer.CLIMBER.setIsClimbing(false)),
            MANUALLY_RAISE_CLIMBER_COMMAND = ClimberCommands.getSetTargetVoltageCommand(ClimberConstants.MANUALLY_RAISE_CLIMBER_VOLTAGE),
            MANUALLY_LOWER_CLIMBER_COMMAND = ClimberCommands.getSetTargetVoltageCommand(ClimberConstants.MANUALLY_LOWER_CLIMBER_VOLTAGE),
            CALCULATE_CAMERA_POSITION_COMMAND = new CameraPositionCalculationCommand(
                    () -> {
                        return RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
                    },
                    Rotation2d.fromDegrees(200),
                    (omegaRadiansPerSecond) -> RobotContainer.SWERVE.selfRelativeDriveWithoutSetpointGeneration(new ChassisSpeeds(0, 0, omegaRadiansPerSecond), null),
                    RobotContainer.SWERVE
            );

    public static final Command
            ENABLE_AUTO_CORAL_INTAKE_COMMAND = new InstantCommand(() -> CoralCollectionCommands.SHOULD_INTAKE_CORAL_AUTONOMOUSLY = true).ignoringDisable(true),
            DISABLE_AUTO_CORAL_INTAKE_COMMAND = new InstantCommand(() -> CoralCollectionCommands.SHOULD_INTAKE_CORAL_AUTONOMOUSLY = false).ignoringDisable(true),
            ENABLE_AUTONOMOUS_REEF_SCORING_COMMAND = new InstantCommand(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY = true).ignoringDisable(true),
            DISABLE_AUTONOMOUS_REEF_SCORING_COMMAND = new InstantCommand(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY = false).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_GRIPPER_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L1_GRIPPER).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_CORAL_INTAKE_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L1_CORAL_INTAKE).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_LEVEL_L2_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L2).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_LEVEL_L3_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L3).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_LEVEL_L4_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L4).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_2_OCLOCK_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_2_OCLOCK).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_4_OCLOCK_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_4_OCLOCK).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_6_OCLOCK_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_6_OCLOCK).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_8_OCLOCK_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_8_OCLOCK).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_10_OCLOCK_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_10_OCLOCK).ignoringDisable(true),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_12_OCLOCK_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_12_OCLOCK).ignoringDisable(true),
            SET_TARGET_REEF_SCORING_SIDE_LEFT_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_SIDE = FieldConstants.ReefSide.LEFT).ignoringDisable(true),
            SET_TARGET_REEF_SCORING_SIDE_RIGHT_COMMAND = new InstantCommand(() -> CoralPlacingCommands.TARGET_REEF_SCORING_SIDE = FieldConstants.ReefSide.RIGHT).ignoringDisable(true);

    /**
     * Calculates the target drive power from an axis value by dividing it by the shift mode value.
     *
     * @param axisValue the stick's value
     * @return the drive power
     */
    public static double calculateDriveStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER);
    }

    /**
     * Calculates the target rotation power from an axis value by dividing it by the shift mode value.
     *
     * @param axisValue the stick's value
     * @return the rotation power
     */
    public static double calculateRotationStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_ROTATION_SHIFT_POWER);
    }

    /**
     * The shift mode is a mode of the robot that slows down the robot relative to how much the right trigger axis is pressed.
     * This method will take the given power, and slow it down relative to how much the right trigger is pressed.
     *
     * @param minimumPower the minimum amount of power the shift mode can limit (as an absolute number)
     * @return the power to apply to the robot
     */
    public static double calculateShiftModeValue(double minimumPower) {
        final double squaredShiftModeValue = Math.sqrt(DRIVER_CONTROLLER.getRightTriggerAxis());
        final double minimumShiftValueCoefficient = 1 - (1 / minimumPower);

        return 1 - squaredShiftModeValue * minimumShiftValueCoefficient;
    }

    /**
     * Calculates the target rotation value from the joystick's angle. Used for joystick oriented rotation.
     * Joystick oriented rotation is when the robot rotates directly to the joystick's angle.
     *
     * @return the rotation value
     */
    private static FlippableRotation2d calculateJoystickOrientedToReefSectionsTargetAngle() {
        final double
                xPower = DRIVER_CONTROLLER.getRightX(),
                yPower = DRIVER_CONTROLLER.getRightY();

        final double joystickPower = Math.hypot(xPower, yPower);
        if (joystickPower < JOYSTICK_ORIENTED_ROTATION_DEADBAND)
            return null;

        final double targetAngleRadians = Math.atan2(xPower, yPower) + Math.PI;
        return roundAngleToClosestReefAngle(Rotation2d.fromRadians(targetAngleRadians));
    }

    private static FlippableRotation2d roundAngleToClosestReefAngle(Rotation2d targetAngle) {
        Rotation2d closestReefAngle = new Rotation2d();
        double closestReefAngleOffsetDegrees = Double.POSITIVE_INFINITY;

        for (Rotation2d reefClockAngle : FieldConstants.REEF_CLOCK_ANGLES) {
            double angleOffsetDegrees = Math.abs(targetAngle.minus(reefClockAngle).getDegrees());
            if (angleOffsetDegrees < closestReefAngleOffsetDegrees) {
                closestReefAngleOffsetDegrees = angleOffsetDegrees;
                closestReefAngle = reefClockAngle;
            }
        }

        return new FlippableRotation2d(closestReefAngle, true);
    }

    private static double getXPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.cos(povRadians);
    }

    private static double getYPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.sin(-povRadians);
    }
}