package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.CoralAlignmentCommand;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.trigon.hardware.misc.leds.LEDCommands;

/**
 * A class that contains commands for collecting game pieces.
 */
public class CollectionCommands {
    public static boolean SHOULD_ALIGN_TO_CORAL = true;

    public static Command getFloorCoralCollectionCommand() {
        return getInitiateFloorCoralCollectionCommand().unless(RobotContainer.GRIPPER::hasGamePiece).alongWith(
                GeneralCommands.runWhen(GeneralCommands.duplicate(CommandConstants.COLLECTION_RUMBLE_COMMAND), RobotContainer.CORAL_INTAKE::hasGamePiece)
        );
    }

    public static Command getFeederCoralCollectionCommand() {
        return getInitiateFeederCoralCollectionCommand().unless(RobotContainer.GRIPPER::hasGamePiece).alongWith(
                GeneralCommands.runWhen(GeneralCommands.duplicate(CommandConstants.COLLECTION_RUMBLE_COMMAND), RobotContainer.CORAL_INTAKE::hasGamePiece)
        );
    }

    private static Command getInitiateFeederCoralCollectionCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FEEDER)
        ).until(() -> RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece()).finallyDo(
                (interrupted) -> {
                    if (!interrupted)
                        getLoadCoralCommand().schedule();
                }
        );
    }

    private static Command getInitiateFloorCoralCollectionCommand() {
        return new ParallelCommandGroup(
                new CoralAlignmentCommand().onlyIf(() -> SHOULD_ALIGN_TO_CORAL).asProxy(),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED).unless(() -> SHOULD_ALIGN_TO_CORAL),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR)
        ).until(() -> RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece()).finallyDo(
                (interrupted) -> {
//                    if (!interrupted)
                    getLoadCoralCommand().schedule();
                }
        );
    }

    public static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.LOAD_CORAL),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                getCoralIntakeSequnceCommand()
        ).until(RobotContainer.GRIPPER::hasGamePiece).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private static Command getCoralIntakeSequnceCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.PREPARE_FOR_LOADING_WHILE_GAME_PIECE_NOT_DETECTED).until(RobotContainer.CORAL_INTAKE::hasGamePiece),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.PREPARE_FOR_LOADING_WHILE_GAME_PIECE_DETECTED).until(RobotContainer.CORAL_INTAKE::atTargetAngle),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL)
        );
    }
}