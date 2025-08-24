package frc.trigon.robot.subsystems.algaemanipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
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

    public static Command getDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                new StartEndCommand(
                        () -> RobotContainer.ALGAE_MANIPULATOR.setTargetState(AlgaeManipulatorConstants.AlgaeManipulatorState.REST),
                        () -> {
                        },
                        RobotContainer.ALGAE_MANIPULATOR
                ),
                getSetTargetStateCommand(AlgaeManipulatorConstants.AlgaeManipulatorState.OPEN_FOR_GRIPPER),
                () -> RobotContainer.GRIPPER.getCurrentEncoderAngle().getDegrees() < AlgaeManipulatorConstants.MAXIMUM_RESTING_GRIPPER_ANGLE.getDegrees()
        );
    }

    public static Command getOpenCommand() {
        return new StartEndCommand(
                RobotContainer.ALGAE_MANIPULATOR::open,
                RobotContainer.ALGAE_MANIPULATOR::stop,
                RobotContainer.ALGAE_MANIPULATOR
        );
    }

    public static Command getPressLimitSwitchCommand() {
        return new StartEndCommand(
                RobotContainer.ALGAE_MANIPULATOR::closeToLimit,
                RobotContainer.ALGAE_MANIPULATOR::stop,
                RobotContainer.ALGAE_MANIPULATOR
        ).raceWith(
                new WaitUntilCommand(RobotContainer.ALGAE_MANIPULATOR::hasHitReverseLimit),
                new WaitUntilCommand(OperatorConstants.CONTINUE_TRIGGER).andThen(new InstantCommand(RobotContainer.ALGAE_MANIPULATOR::resetPosition))
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