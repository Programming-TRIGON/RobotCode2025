package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.CoralAlignmentCommand;
import frc.trigon.robot.subsystems.algaeintake.AlgaeIntakeCommands;
import frc.trigon.robot.subsystems.algaeintake.AlgaeIntakeConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import org.trigon.hardware.misc.leds.LEDCommands;

/**
 * A class that contains commands for collecting game pieces.
 */
public class CollectionCommands {
    public static Command getCollectAlgaeCommand() {
        return new ParallelCommandGroup(
                AlgaeIntakeCommands.getSetTargetStateCommand(AlgaeIntakeConstants.AlgaeIntakeState.COLLECT),
                LEDCommands.getBlinkingCommand(Color.kAqua, AlgaeIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED)
        );
    }

    public static Command getCollectCoralFromGroundCommand() {
        return new ParallelCommandGroup(
                new CoralAlignmentCommand().onlyIf(() -> CommandConstants.SHOULD_ALIGN_TO_CORAL),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT).unless(() -> CommandConstants.SHOULD_ALIGN_TO_CORAL),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.COLLECTION_LEDS_BLINKING_SPEED)
        ).unless(RobotContainer.CORAL_INTAKE::hasGamePiece).alongWith(CommandConstants.COLLECTION_RUMBLE_COMMAND).onlyIf(RobotContainer.CORAL_INTAKE::hasGamePiece);
    }
}