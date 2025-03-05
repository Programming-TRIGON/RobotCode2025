package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;
import java.util.function.Supplier;

public class ElevatorCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.ELEVATOR::setTargetPositionRotations,
                false,
                Set.of(RobotContainer.ELEVATOR),
                "Debugging/ElevatorTargetPositionRotations"
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.sysIdDrive(targetVoltage),
                RobotContainer.ELEVATOR::stop
        );
    }

    public static Command getSetTargetStateCommand(Supplier<ElevatorConstants.ElevatorState> targetStateSupplier) {
        return getWaitUntilAllowedToMoveToPositionCommand(
                RobotContainer.ELEVATOR.metersToRotations(targetStateSupplier.get().targetPositionMeters)
        ).andThen(new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetStateSupplier.get()),
                () -> {
                },
                RobotContainer.ELEVATOR
        ));
    }

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return getWaitUntilAllowedToMoveToPositionCommand(
                RobotContainer.ELEVATOR.metersToRotations(targetState.targetPositionMeters)
        ).andThen(new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetState),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        ));
    }

    public static Command getSetTargetPositionCommand(double targetPositionRotations) {
        return getWaitUntilAllowedToMoveToPositionCommand(
                targetPositionRotations
        ).andThen(new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetPositionRotations(targetPositionRotations),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        ));
    }

    private static Command getWaitUntilAllowedToMoveToPositionCommand(double targetPositionRotations) {
        return new WaitUntilCommand(() -> RobotContainer.ELEVATOR.isAllowedToMoveToPosition(targetPositionRotations));
    }
}
