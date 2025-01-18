package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

import java.util.function.Supplier;

public class ElevatorCommands {
    public static Command setDebuggingCommand() {
        return new NetworkTablesCommand(
                ElevatorCommands::getSetTargetPositionCommand,
                false,
                "Debugging/ElevatorTargetPositionMeters"
        );
    }

    public static Command getSetTargetStateCommand(Supplier<ElevatorConstants.ElevatorState> targetStateSupplier) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetStateSupplier.get()),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetState),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }

    public static Command getSetTargetPositionCommand(double targetPositionRotations) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetPositionRotations(targetPositionRotations),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }
}
