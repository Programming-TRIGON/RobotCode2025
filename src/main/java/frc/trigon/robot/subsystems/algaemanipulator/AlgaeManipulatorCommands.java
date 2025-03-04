package frc.trigon.robot.subsystems.algaemanipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class AlgaeManipulatorCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double targetAngleDegrees) -> RobotContainer.ALGAE_MANIPULATOR.setTargetAngle(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.ALGAE_MANIPULATOR),
                "Debugging/AlgaeManipulatorTargetAngleDegrees"
        );
    }

    public static Command getSetTargetStateCommand(AlgaeManipulatorConstants.AlgaeManipulatorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ALGAE_MANIPULATOR.setTargetState(targetState),
                RobotContainer.ALGAE_MANIPULATOR::stop,
                RobotContainer.ALGAE_MANIPULATOR
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.ALGAE_MANIPULATOR.setTargetAngle(targetAngle),
                RobotContainer.ALGAE_MANIPULATOR::stop,
                RobotContainer.ALGAE_MANIPULATOR
        );
    }
}