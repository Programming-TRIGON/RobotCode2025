package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.ReefAlignmentCommand;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;

public class PlacementCommands {
    private static boolean SHOULD_ALIGN_TO_REEF = true;

    public static Command placeCoral(ElevatorConstants.ElevatorState targetReefLevel, int reefSideInAnalogClock) {
        return new SequentialCommandGroup(
                ReefAlignmentCommand.alignToReef(reefSideInAnalogClock).onlyIf(() -> SHOULD_ALIGN_TO_REEF),
                getCoralLevel(targetReefLevel)
        );
    }

    private static Command getCoralLevel(ElevatorConstants.ElevatorState targetReefLevel) {
        return switch (targetReefLevel) {
            case SCORE_L1 -> getCoralScoreL1();
            case SCORE_L2 -> getCoralScoreL2();
            case SCORE_L3 -> getCoralScoreL3();
            case SCORE_L4 -> getCoralScoreL4();
            default -> null;
        };
    }

    private static Command getCoralScoreL1() {
        return new SequentialCommandGroup(
                getTargetReefLevelPlacement(ElevatorConstants.ElevatorState.SCORE_L1, GripperConstants.GripperState.SCORE_L1)
        );
    }

    private static Command getCoralScoreL2() {
        return new SequentialCommandGroup(
                getTargetReefLevelPlacement(ElevatorConstants.ElevatorState.SCORE_L2, GripperConstants.GripperState.SCORE_L3_OR_L2)
        );
    }

    private static Command getCoralScoreL3() {
        return new SequentialCommandGroup(
                getTargetReefLevelPlacement(ElevatorConstants.ElevatorState.SCORE_L3, GripperConstants.GripperState.SCORE_L3_OR_L2)
        );
    }

    private static Command getCoralScoreL4() {
        return new SequentialCommandGroup(
                getTargetReefLevelPlacement(ElevatorConstants.ElevatorState.SCORE_L4, GripperConstants.GripperState.SCORE_L4)
        );
    }

    private static Command getTargetReefLevelPlacement(ElevatorConstants.ElevatorState targetCoralLevel, GripperConstants.GripperState targetCoralLevelGripperState) {
        return new SequentialCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(targetCoralLevel).until(RobotContainer.ELEVATOR::atTargetState),
                GripperCommands.getSetTargetStateCommand(targetCoralLevelGripperState).until(() -> RobotContainer.GRIPPER.atState(targetCoralLevelGripperState)),
                GeneralCommands.runWhen(GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.REST), () -> !RobotContainer.GRIPPER.hasGamePiece()),
                GeneralCommands.runWhen(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST), () -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.REST)
                )
        );
    }
}
