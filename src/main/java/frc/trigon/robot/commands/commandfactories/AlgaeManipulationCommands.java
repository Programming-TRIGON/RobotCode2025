package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        );
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
        return GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.ALGAE_COLLECTION_ANGLE, GripperConstants.ALGAE_COLLECTION_CURRENT);
    }
}
