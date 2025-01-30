package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.CoralAlignmentCommand;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.trigon.hardware.misc.leds.LEDCommands;

public class CoralCollectionCommands {
    public static boolean SHOULD_ALIGN_TO_CORAL = true;

    public static Command getFloorCoralCollectionCommand() {
        return getInitiateFloorCoralCollectionCommand().unless(RobotContainer.GRIPPER::hasGamePiece);
    }

    public static Command getFeederCoralCollectionCommand() {
        return getInitiateFeederCoralCollectionCommand().unless(RobotContainer.GRIPPER::hasGamePiece);
    }

    private static Command getInitiateFeederCoralCollectionCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FEEDER),
                GeneralCommands.runWhen(GeneralCommands.duplicate(CommandConstants.COLLECTION_RUMBLE_COMMAND), RobotContainer.CORAL_INTAKE::hasGamePiece),
                getScheduleCoralLoadingWhenCollectedCommand()
        );
    }

    private static Command getInitiateFloorCoralCollectionCommand() {
        return new ParallelCommandGroup(
                new CoralAlignmentCommand().onlyIf(() -> SHOULD_ALIGN_TO_CORAL).asProxy(),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED).unless(() -> SHOULD_ALIGN_TO_CORAL),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR),
                GeneralCommands.runWhen(GeneralCommands.duplicate(CommandConstants.COLLECTION_RUMBLE_COMMAND), RobotContainer.CORAL_INTAKE::hasGamePiece),
                getScheduleCoralLoadingWhenCollectedCommand()
        );
    }

    private static Command getScheduleCoralLoadingWhenCollectedCommand() {
        return GeneralCommands.runWhen(getScheduleCoralLoadingCommand(), CoralCollectionCommands::didCollectCoral);
    }

    private static boolean didCollectCoral() {
        return RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece();
    }

    private static Command getScheduleCoralLoadingCommand() {
        return new InstantCommand(() -> {
            if (CoralPlacingCommands.TARGET_SCORING_LEVEL == CoralPlacingCommands.ScoringLevel.L1_CORAL_INTAKE)
                getCenterCoralInIntakeCommand().schedule();
            else
                getLoadCoralCommand().schedule();
        });
    }

    private static Command getCenterCoralInIntakeCommand() {
        return CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.CENTER_CORAL)
                .until(RobotContainer.CORAL_INTAKE::hasGamePiece)
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    public static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.LOAD_CORAL),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                getCoralIntakeLoadingSequnceCommand()
        ).until(CoralCollectionCommands::shouldStopLoadingCoral).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private static boolean shouldStopLoadingCoral() {
        return RobotContainer.GRIPPER.hasGamePiece() || OperatorConstants.CONTINUE_SCORING_TRIGGER.getAsBoolean();
    }

    private static Command getCoralIntakeLoadingSequnceCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.PREPARE_FOR_LOADING_WHILE_GAME_PIECE_NOT_DETECTED).until(RobotContainer.CORAL_INTAKE::hasGamePiece),
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL).until(RobotContainer.CORAL_INTAKE::atTargetAngle),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL)
        );
    }
}