package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.subsystems.algaeintake.AlgaeIntakeCommands;
import frc.trigon.robot.subsystems.algaeintake.AlgaeIntakeConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import org.trigon.hardware.misc.leds.LEDCommands;

/**
 * A class that commands for collecting game pieces.
 */
public class CollectionCommands {
    public static Command getCollectAlgaeCommand() {
        return new ParallelCommandGroup(
                AlgaeIntakeCommands.getSetTargetStateCommand(AlgaeIntakeConstants.AlgaeIntakeState.COLLECT),
                LEDCommands.getBlinkingCommand(Color.kAqua, AlgaeIntakeConstants.BLINKING_SPEED)
        );
    }

    public static Command getCollectCoralFromFeederCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FEEDER),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.BLINKING_SPEED)
        ).unless(RobotContainer.CORAL_INTAKE::hasGamePiece).alongWith(CommandConstants.RUMBLE_COMMAND);
    }

    public static Command getCollectCoralFromGroundCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT),
                LEDCommands.getBlinkingCommand(Color.kAqua, CoralIntakeConstants.BLINKING_SPEED)
        ).unless(RobotContainer.CORAL_INTAKE::hasGamePiece).alongWith(CommandConstants.RUMBLE_COMMAND);
    }
}