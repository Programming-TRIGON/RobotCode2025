package frc.trigon.robot.subsystems.coralintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

public class CoralIntakeCommands {
    public static Command getIntakeAndFunnelDebuggingCommand() {
        return new NetworkTablesCommand(
                CoralIntakeCommands::getSetTargetVoltageCommand,
                false,
                "Debugging/TargetCoralIntakeVoltage",
                "Debugging/TargetCoralFunnelVoltage"
        );
    }

    public static Command getAngleDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> getSetTargetAngleCommand(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                "Debugging/TargetCoralAngle"
        );
    }

    public static Command getAngleMotorCalculateGearRatioCommand() {
        return new GearRatioCalculationCommand(
                CoralIntakeConstants.ANGLE_MOTOR,
                CoralIntakeConstants.ENCODER,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState state) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(state),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetVoltageCommand(double intakeVoltage, double funnelVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetVoltage(intakeVoltage, funnelVoltage),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetAngle(targetAngle),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }
}