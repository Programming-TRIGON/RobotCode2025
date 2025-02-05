package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

public class ClimberCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetPositionRotations, isAffectedByRobotWeight) -> getSetTargetPositionCommand(targetPositionRotations, isAffectedByRobotWeight == 1),
                true,
                "Debugging/ClimberTargetPositionRotations",
                "Debugging/ClimberIsAffectedByRobotWeight"
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetPositionCommand(double targetPositionRotations, boolean isAffectedByRobotWeight) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetPosition(targetPositionRotations, isAffectedByRobotWeight),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.CLIMBER::stop,
                () -> {
                },
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetVoltageCommand(double voltage) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.sysIdDrive(voltage),
                () -> {
                },
                RobotContainer.CLIMBER
        );
    }
}