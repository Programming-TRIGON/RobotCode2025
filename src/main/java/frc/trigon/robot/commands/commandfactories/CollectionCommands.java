package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    public static boolean IS_LOADING_CORAL = false;

    public static Command getCoralCollectionCommand() {
        return getInitiateCoralCollectionCommand().unless(RobotContainer.CORAL_INTAKE::hasGamePiece).alongWith(CommandConstants.COLLECTION_RUMBLE_COMMAND);
    }

    private static Command getInitiateCoralCollectionCommand() {
        return new ParallelCommandGroup(
                new CoralAlignmentCommand().onlyIf(() -> SHOULD_ALIGN_TO_CORAL),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED).unless(() -> SHOULD_ALIGN_TO_CORAL),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT)
        ).until(() -> RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece()).finallyDo(
                (interrupted) -> {
                    if (!interrupted)
                        getLoadCoralCommand().schedule();
                }
        ).unless(() -> RobotContainer.CORAL_INTAKE.hasGamePiece() || RobotContainer.CORAL_INTAKE.hasGamePiece());
    }

    private static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> IS_LOADING_CORAL = true),
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.LOAD_CORAL),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                getCoralIntakeCommand()
        ).until(RobotContainer.CORAL_INTAKE::hasGamePiece).finallyDo(
                () -> IS_LOADING_CORAL = false
        );
    }

    private static Command getCoralIntakeCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.PREPARE_FOR_LOADING).until(RobotContainer.CORAL_INTAKE::hasGamePiece),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.PREPARE_FOR_LOADING_WHILE_GAME_PIECE_DETECTED).alongWith(CommandConstants.COLLECTION_RUMBLE_COMMAND).until(RobotContainer.CORAL_INTAKE::atTargetAngle),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL)
        );
    }
}