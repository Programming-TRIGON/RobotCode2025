package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;

public class AlgaeManipulationCommands {
    public static Command getCollectAlgaeFromReefCommand() {
        return new ConditionalCommand(
                getCollectAlgaeFromL2Command(),
                getCollectAlgaeFromL3Command(),
                () -> CoralPlacingCommands.TARGET_SCORING_LEVEL == CoralPlacingCommands.ScoringLevel.L2
        ).alongWith(
                new WaitUntilCommand(OperatorConstants.NET_TRIGGER).andThen(new InstantCommand(() -> getScoreInNetCommand().schedule()))
        );
    }

    private static Command getScoreInNetCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_NET),
                GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.PREPARE_FOR_SCORING_ALGAE_IN_NET).until(OperatorConstants.CONTINUE_SCORING_TRIGGER).andThen(
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_ALGAE_IN_NET)
                )
        ).onlyWhile(OperatorConstants.COLLECT_ALGAE_TRIGGER);
    }

    private static Command getCollectAlgaeFromL3Command() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.COLLECT_ALGAE_FROM_L3),
                getGripAlgaeCommand()
        );
    }

    private static Command getCollectAlgaeFromL2Command() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                getGripAlgaeCommand()
        );
    }

    private static Command getGripAlgaeCommand() {
        return GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_REEF);
    }
}
