package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.trigon.hardware.misc.leds.LEDCommands;
import org.trigon.hardware.misc.leds.LEDStrip;


public class CoralCollectionCommands {
    public static boolean SHOULD_INTAKE_CORAL_AUTONOMOUSLY = true;

    public static Command getFeederCoralCollectionFromGripperCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.COLLECT_CORAL_FROM_FEEDER),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
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
                LEDCommands.getAnimateCommand(LEDConstants.CORAL_STATION_INTAKE_SETTINGS, LEDStrip.LED_STRIPS),
                getScheduleCoralLoadingWhenCollectedCommand()
        );
    }

    private static Command getInitiateFloorCoralCollectionCommand() {
        return new ParallelCommandGroup(
//                GeneralCommands.getContinuousConditionalCommand(
//                        LEDCommands.getAnimateCommand(LEDConstants.GROUND_INTAKE_WITH_CORAL_VISIBLE_TO_CAMERA_SETTINGS, LEDStrip.LED_STRIPS),
//                        LEDCommands.getAnimateCommand(LEDConstants.GROUND_INTAKE_WITHOUT_CORAL_VISIBLE_TO_CAMERA_SETTINGS, LEDStrip.LED_STRIPS),
//                        () -> CameraConstants.OBJECT_DETECTION_CAMERA.hasTargets(SimulatedGamePieceConstants.GamePieceType.CORAL)
//                ).until(CoralCollectionCommands::didCollectCoral),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR),
                getScheduleCoralLoadingWhenCollectedCommand()
        );
    }

    private static Command getScheduleCoralLoadingWhenCollectedCommand() {
        return GeneralCommands.runWhen(getCollectionConfirmationCommand().alongWith(getScheduleCoralLoadingCommand()), CoralCollectionCommands::didCollectCoral);
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
                getCoralIntakeLoadingSequenceCommand()
        ).unless(CoralCollectionCommands::shouldStopLoadingCoral).until(CoralCollectionCommands::shouldStopLoadingCoral).andThen(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF).until(() -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF))
        );
    }

    public static Command getUnloadCoralCommand() {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        GripperCommands.getPrepareForStateCommand(GripperConstants.GripperState.UNLOAD_CORAL).until(() -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.UNLOAD_CORAL)),
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.UNLOAD_CORAL)
                ),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.UNLOAD_CORAL_FROM_GRIPPER)
        ).unless(RobotContainer.CORAL_INTAKE::hasGamePiece).until(RobotContainer.CORAL_INTAKE::hasGamePiece);
    }

    private static boolean shouldStopLoadingCoral() {
        return RobotContainer.GRIPPER.hasGamePiece() || OperatorConstants.CONTINUE_TRIGGER.getAsBoolean();
    }

    private static Command getCoralIntakeLoadingSequenceCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getCenterCoralWithPulsingCommand().until(RobotContainer.CORAL_INTAKE::hasGamePiece),
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK).until(RobotContainer.CORAL_INTAKE::atTargetAngle),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK).raceWith(new WaitUntilCommand(() -> !RobotContainer.CORAL_INTAKE.hasGamePiece()).andThen(new WaitCommand(0.4))),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_NOT_SEEING_GAME_PIECE_WITH_BEAM_BREAK)
        );
    }

    private static Command getCollectionConfirmationCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> OperatorConstants.DRIVER_CONTROLLER.rumble(CoralIntakeConstants.COLLECTION_RUMBLE_DURATION_SECONDS, CoralIntakeConstants.COLLECTION_RUMBLE_POWER)),
                LEDCommands.getAnimateCommand(LEDConstants.INTAKE_CONFIRMATION_SETTINGS, LEDStrip.LED_STRIPS).withTimeout(LEDConstants.RELEASE_CORAL_TIMEOUT_SECONDS)
        );
    }

    private static boolean didCollectCoral() {
        return RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece();
    }
}