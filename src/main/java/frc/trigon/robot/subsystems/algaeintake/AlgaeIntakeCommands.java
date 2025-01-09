package frc.trigon.robot.subsystems.algaeintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class AlgaeIntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double[] targetState) -> RobotContainer.ALGAE_INTAKE.setTargetState(targetState[0], Rotation2d.fromDegrees(targetState[1])),
                false,
                Set.of(RobotContainer.ALGAE_INTAKE),
                "Debugging/AlgaeIntakeVoltage",
                "Debugging/AlgaeIntakeAngle"
        );
    }

    public static Command getCalculateGearRatioCommand() {
        return new GearRatioCalculationCommand(
                AlgaeIntakeConstants.ANGLE_MOTOR,
                AlgaeIntakeConstants.ANGLE_ENCODER,
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

    public static Command getSetTargetStateCommand(double targetVoltage, Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.ALGAE_INTAKE.setTargetState(targetVoltage, targetAngle),
                RobotContainer.ALGAE_INTAKE::stopIntakeMotor,
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