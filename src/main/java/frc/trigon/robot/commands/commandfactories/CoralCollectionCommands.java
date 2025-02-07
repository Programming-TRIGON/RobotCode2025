package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.CoralAlignmentCommand;
import frc.trigon.robot.constants.FieldConstants;
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

    public static Command getCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getFloorCoralCollectionCommand(),
                new InstantCommand(() -> {
                    CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L3;
                    CoralPlacingCommands.TARGET_REEF_SCORING_SIDE = FieldConstants.ReefSide.LEFT;
                    CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_6_OCLOCK;
                }),
                new WaitCommand(0.2),
                CoralPlacingCommands.getScoreInReefCommand().raceWith(new WaitUntilCommand(() -> RobotContainer.SWERVE.atPose(CoralPlacingCommands.calculateTargetScoringPose())).andThen(new WaitCommand(0.3))),
                CoralCollectionCommands.getFloorCoralCollectionCommand(),
                new InstantCommand(() -> {
                    CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L3;
                    CoralPlacingCommands.TARGET_REEF_SCORING_SIDE = FieldConstants.ReefSide.RIGHT;
                    CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_6_OCLOCK;
                }),
                new WaitCommand(0.2),
                CoralPlacingCommands.getScoreInReefCommand().raceWith(new WaitUntilCommand(() -> RobotContainer.SWERVE.atPose(CoralPlacingCommands.calculateTargetScoringPose())).andThen(new WaitCommand(0.3))),
                CoralCollectionCommands.getFloorCoralCollectionCommand(),
                new InstantCommand(() -> {
                    CoralPlacingCommands.TARGET_SCORING_LEVEL = CoralPlacingCommands.ScoringLevel.L2;
                    CoralPlacingCommands.TARGET_REEF_SCORING_SIDE = FieldConstants.ReefSide.RIGHT;
                    CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_6_OCLOCK;
                }),
                new WaitCommand(0.2),
                CoralPlacingCommands.getScoreInReefCommand().raceWith(new WaitUntilCommand(() -> RobotContainer.SWERVE.atPose(CoralPlacingCommands.calculateTargetScoringPose())).andThen(new WaitCommand(0.3)))
        );
    }

    public static Command getFeederCoralCollectionFromGripperCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.COLLECT_CORAL_FROM_FEEDER),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
//                getCollectionRumbleWhenHasGamePieceCommand(),
                getScheduleCoralLoadingWhenCollectedCommand()
        ).until(RobotContainer.GRIPPER::hasGamePiece);
    }

    public static Command getFloorCoralCollectionCommand() {
        return getInitiateFloorCoralCollectionCommand().unless(RobotContainer.GRIPPER::hasGamePiece);
    }

    public static Command getFeederCoralCollectionCommand() {
        return getInitiateFeederCoralCollectionCommand().unless(RobotContainer.GRIPPER::hasGamePiece);
    }

    private static Command getInitiateFeederCoralCollectionCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FEEDER),
//                getCollectionRumbleWhenHasGamePieceCommand(),
                getScheduleCoralLoadingWhenCollectedCommand()
        );
    }

    private static Command getInitiateFloorCoralCollectionCommand() {
        return new ParallelCommandGroup(
                new CoralAlignmentCommand().onlyIf(() -> SHOULD_ALIGN_TO_CORAL).asProxy(),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED).unless(() -> SHOULD_ALIGN_TO_CORAL),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR),
//                getCollectionRumbleWhenHasGamePieceCommand(),
                getScheduleCoralLoadingWhenCollectedCommand()
        );
    }

    private static Command getScheduleCoralLoadingWhenCollectedCommand() {
        return GeneralCommands.runWhen(getCollectionRumbleCommand().andThen(getScheduleCoralLoadingCommand()), CoralCollectionCommands::didCollectCoral);
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
                .until(RobotContainer.CORAL_INTAKE::hasGamePiece);
    }

    public static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.LOAD_CORAL),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                getCoralIntakeLoadingSequnceCommand()
        ).unless(CoralCollectionCommands::shouldStopLoadingCoral).until(CoralCollectionCommands::shouldStopLoadingCoral);
    }

    public static Command getUnloadCoralCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.UNLOAD_CORAL),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.UNLOAD_CORAL_FROM_GRIPPER)
        ).unless(RobotContainer.CORAL_INTAKE::hasGamePiece).until(RobotContainer.CORAL_INTAKE::hasGamePiece);
    }

    private static boolean shouldStopLoadingCoral() {
        return RobotContainer.GRIPPER.hasGamePiece() || OperatorConstants.CONTINUE_SCORING_TRIGGER.getAsBoolean();
    }

    private static Command getCoralIntakeLoadingSequnceCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.PREPARE_FOR_LOADING_TO_GRIPPER_WHILE_GAME_PIECE_NOT_DETECTED).until(RobotContainer.CORAL_INTAKE::hasGamePiece),
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE).until(RobotContainer.CORAL_INTAKE::atTargetAngle),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE).onlyWhile(RobotContainer.CORAL_INTAKE::hasGamePiece),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_NOT_SEEING_GAME_PIECE)
        );
    }

    private static Command getCollectionRumbleWhenHasGamePieceCommand() {
        return GeneralCommands.runWhen(getCollectionRumbleCommand(), CoralCollectionCommands::didCollectCoral);
    }

    private static Command getCollectionRumbleCommand() {
        return new InstantCommand(
                () -> OperatorConstants.DRIVER_CONTROLLER.rumble(CoralIntakeConstants.COLLECTION_RUMBLE_DURATION_SECONDS, CoralIntakeConstants.COLLECTION_RUMBLE_POWER)
        );
    }

    private static boolean didCollectCoral() {
        return RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece();
    }
}