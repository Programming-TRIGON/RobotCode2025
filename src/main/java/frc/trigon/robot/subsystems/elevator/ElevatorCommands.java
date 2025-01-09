package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ElevatorCommands {
    public static Command getSetTargetPositionCommand(double targetPositionMeters) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetPosition(targetPositionMeters),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetElevatorState(targetState),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }
}
