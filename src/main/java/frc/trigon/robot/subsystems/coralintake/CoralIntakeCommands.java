package frc.trigon.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class CoralIntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double[] targetStates) -> RobotContainer.CORAL_INTAKE.setTargetState(
                        targetStates[0],
                        targetStates[1],
                        targetStates[2]
                ),
                false,
                Set.of(RobotContainer.CORAL_INTAKE),
                "Debugging/TargetCoralIntakeVoltage",
                "Debugging/TargetCoralIntakeFunnelVoltage",
                "Debugging/TargetCoralIntakeElevatorPositionRotations"
        );
    }

    public static Command getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState targetState) {
        if (targetState == CoralIntakeConstants.CoralIntakeState.COLLECT)
            return getCollectionCommand();
        
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(targetState),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetStateCommand(double targetIntakeVoltage, double targetFunnelVoltage, double targetPositionRotations) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(targetIntakeVoltage, targetFunnelVoltage, targetPositionRotations),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.CORAL_INTAKE::stop,
                () -> {
                },
                RobotContainer.CORAL_INTAKE
        );
    }

    private static Command getCollectionCommand() {
        return new FunctionalCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(CoralIntakeConstants.CoralIntakeState.COLLECT),
                () -> {
                },
                (interrupted) -> RobotContainer.CORAL_INTAKE.stop(),
                RobotContainer.CORAL_INTAKE::hasGamePiece,
                RobotContainer.CORAL_INTAKE
        );
    }
}