package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;

public class PlacementCommands {
    public static boolean SHOULD_ALIGN_TO_CORAL = true;

    public static Command getCoralScoreL1() {
        return new ParallelCommandGroup(
                getTargetReefLevelPlacement(ElevatorConstants.ElevatorState.SCORE_L1, GripperConstants.GripperState.SCORE_L1)
        );
    }

    public static Command getCoralScoreL2() {
        return new ParallelCommandGroup(
                getTargetReefLevelPlacement(ElevatorConstants.ElevatorState.SCORE_L2, GripperConstants.GripperState.SCORE_L3_OR_L2)
        );
    }

    public static Command getCoralScoreL3() {
        return new ParallelCommandGroup(
                getTargetReefLevelPlacement(ElevatorConstants.ElevatorState.SCORE_L3, GripperConstants.GripperState.SCORE_L3_OR_L2)
        );
    }

    public static Command getCoralScoreL4() {
        return new ParallelCommandGroup(
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
