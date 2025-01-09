package frc.trigon.robot.subsystems.algaeintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

public class AlgaeIntakeCommands {
    public static Command getIntakeDebuggingCommand() {
        return new NetworkTablesCommand(
                AlgaeIntakeCommands::getSetTargetVoltageCommand,
                false,
                "AlgaeIntake/TargetVoltage"
        );
    }

    public static Command getAngleDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> getSetTargetAngleCommand(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                "AlgaeIntake/TargetAngle"
        );
    }

    public static Command getCalculateGearRatioCommand() {
        return new GearRatioCalculationCommand(
                () -> RobotContainer.ALGAE_INTAKE.getRotorPosition().getDegrees(),
                () -> RobotContainer.ALGAE_INTAKE.getEncoderPosition().getDegrees(),
                RobotContainer.ALGAE_INTAKE::setTargetVoltage,
                RobotContainer.ALGAE_INTAKE
        );
    }

    public static Command getSetTargetStateCommand(AlgaeIntakeConstants.AlgaeIntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ALGAE_INTAKE.setTargetState(targetState),
                RobotContainer.ALGAE_INTAKE::stop,
                RobotContainer.ALGAE_INTAKE
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.ALGAE_INTAKE.setTargetVoltage(targetVoltage),
                RobotContainer.ALGAE_INTAKE::stopIntakeMotor,
                RobotContainer.ALGAE_INTAKE
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.ALGAE_INTAKE.setTargetAngle(targetAngle),
                RobotContainer.ALGAE_INTAKE::stopAngleMotor,
                RobotContainer.ALGAE_INTAKE
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.ALGAE_INTAKE::stop,
                () -> {
                },
                RobotContainer.ALGAE_INTAKE
        );
    }
}