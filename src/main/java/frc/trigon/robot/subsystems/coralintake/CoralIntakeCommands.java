package frc.trigon.robot.subsystems.coralintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class CoralIntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double[] targetStates) -> RobotContainer.CORAL_INTAKE.setTargetState(
                        targetStates[0],
                        targetStates[1],
                        Rotation2d.fromDegrees(targetStates[2])
                ),
                false,
                Set.of(RobotContainer.CORAL_INTAKE),
                "Debugging/TargetCoralIntakeVoltage",
                "Debugging/TargetCoralIntakeFunnelVoltage",
                "Debugging/TargetCoralIntakeAngle"
        );
    }

    public static Command getAngleMotorCalculateGearRatioCommand() {
        return new GearRatioCalculationCommand(
                CoralIntakeConstants.MASTER_ANGLE_MOTOR,
                CoralIntakeConstants.ANGLE_ENCODER,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(targetState),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetStateCommand(double targetIntakeVoltage, double targetFunnelVoltage, Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(targetIntakeVoltage, targetFunnelVoltage, targetAngle),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }
}