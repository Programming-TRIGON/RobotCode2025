package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.algaemanipulator.AlgaeManipulatorCommands;
import frc.trigon.robot.subsystems.algaemanipulator.AlgaeManipulatorConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;

public class AlgaeManipulationCommands {
    public static Command getCollectAlgaeFromFloorCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_ALGAE_FROM_FLOOR)
        );
    }

    public static Command getCollectAlgaeFromReefCommand() {
        return new ParallelCommandGroup(
                getOpenElevatorForAlgaeCommand(),
                getGripAlgaeCommand()
        ).until(OperatorConstants.SCORE_IN_NET_TRIGGER).andThen(
                getScoreInNetCommand()
        );
    }

    private static Command getOpenElevatorForAlgaeCommand() {
        return new ConditionalCommand(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.COLLECT_ALGAE_FROM_L3),
                () -> CoralPlacingCommands.TARGET_SCORING_LEVEL == CoralPlacingCommands.ScoringLevel.L2
        );
    }

    private static Command getScoreInNetCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_NET),
                AlgaeManipulatorCommands.getSetTargetStateCommand(AlgaeManipulatorConstants.AlgaeManipulatorState.HOLD_ALGAE),
                new SequentialCommandGroup(
                        GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.PREPARE_FOR_SCORING_ALGAE_IN_NET),
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_ALGAE_IN_NET)
                )
        );
    }

    private static Command getGripAlgaeCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_REEF).raceWith(new WaitCommand(1).andThen(new WaitUntilCommand(RobotContainer.GRIPPER::isMovingSlowly))),
                GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.HOLD_ALGAE)
        ).alongWith(
                AlgaeManipulatorCommands.getSetTargetStateCommand(AlgaeManipulatorConstants.AlgaeManipulatorState.HOLD_ALGAE)
        );
    }
}