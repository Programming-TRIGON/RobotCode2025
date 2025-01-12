package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

public class ElevatorCommands {
    public static Command setDebuggingCommand() {
        return new NetworkTablesCommand(
                ElevatorCommands::getSetTargetPositionCommand,
                false,
                ""
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
                () -> RobotContainer.ELEVATOR.setTargetPositionMeters(targetPositionRotations),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }
}
