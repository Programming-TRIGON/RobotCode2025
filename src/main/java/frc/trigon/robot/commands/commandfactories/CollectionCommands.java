package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.CoralAlignmentCommand;
import frc.trigon.robot.subsystems.algaeintake.AlgaeIntakeCommands;
import frc.trigon.robot.subsystems.algaeintake.AlgaeIntakeConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.trigon.hardware.misc.leds.LEDCommands;

/**
 * A class that contains commands for collecting game pieces.
 */
public class CollectionCommands {
    public static boolean SHOULD_ALIGN_TO_CORAL = true;
    public static boolean HAS_INITIATED_COLLECTION = false;

    public static Command getAlgaeCollectionCommand() {
        return new ParallelCommandGroup(
                AlgaeIntakeCommands.getSetTargetStateCommand(AlgaeIntakeConstants.AlgaeIntakeState.COLLECT),
                LEDCommands.getBlinkingCommand(Color.kAqua, AlgaeIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED)
        );
    }

    public static Command getCoralCollectionCommand() {
        return new SequentialCommandGroup(
                getInitiateCollectionCommand(),
                new InstantCommand(() -> HAS_INITIATED_COLLECTION = true),
                GeneralCommands.duplicate(CommandConstants.COLLECTION_RUMBLE_COMMAND)
        ).unless(RobotContainer.CORAL_INTAKE::hasGamePiece).alongWith(CommandConstants.COLLECTION_RUMBLE_COMMAND).onlyIf(RobotContainer.CORAL_INTAKE::hasGamePiece);
    }

    public static Command getCoralCollectionOnFalseCommand() {
        if (!HAS_INITIATED_COLLECTION)
            return RobotContainer.CORAL_INTAKE.getDefaultCommand();
        return RobotContainer.CORAL_INTAKE.getCurrentCommand();
    }

    private static Command getInitiateCollectionCommand() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> HAS_INITIATED_COLLECTION = false),
                new CoralAlignmentCommand().onlyIf(() -> SHOULD_ALIGN_TO_CORAL),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED).unless(() -> SHOULD_ALIGN_TO_CORAL),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT),
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.LOAD_CORAL)
        ).until(RobotContainer.CORAL_INTAKE::isEarlyCoralCollectionDetected);
    }

    private static Command getRetractCoralIntakeAndFeedToGripperCommand() {
        return new SequentialCommandGroup(
                getRetractCoralIntakeCommand(),
                GeneralCommands.runWhen(CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.FEED), RobotContainer.CORAL_INTAKE::atTargetAngle),
                GeneralCommands.runWhen(CoralIntakeCommands.getStopCommand(), RobotContainer.GRIPPER::hasGamePiece)
        );
    }

    private static Command getRetractCoralIntakeCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.RETRACT).until(() -> RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.RETRACT)),
                CoralIntakeCommands.getStopCommand(),
                () -> RobotContainer.CORAL_INTAKE.hasGamePiece() && RobotContainer.CORAL_INTAKE.atTargetAngle()
        ).asProxy();
    }
}