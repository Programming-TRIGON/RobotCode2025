package frc.trigon.robot.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.trigon.robot.commands.commandfactories.CoralPlacingCommands;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.trigon.utilities.flippable.FlippablePose2d;

import java.util.function.Supplier;

public class ReefChooser {
    private final CommandGenericHID hid;
    private FieldConstants.ReefClockPosition clockPosition = FieldConstants.ReefClockPosition.REEF_6_OCLOCK;
    private CoralPlacingCommands.ScoringLevel scoringLevel = CoralPlacingCommands.ScoringLevel.L4;
    private FieldConstants.ReefSide reefSide = FieldConstants.ReefSide.LEFT;

    public ReefChooser(int port) {
        hid = new CommandGenericHID(port);

        new WaitCommand(3).andThen(this::configureBindings).ignoringDisable(true).schedule();
    }

    public CoralPlacingCommands.ScoringLevel getScoringLevel() {
        return scoringLevel;
    }

    public FieldConstants.ReefClockPosition getClockPosition() {
        return clockPosition;
    }

    public FieldConstants.ReefSide getReefSide() {
        return reefSide;
    }

    public FlippablePose2d calculateTargetScoringPose() {
        return scoringLevel.calculateTargetPlacingPosition(clockPosition, reefSide);
    }

    public GripperConstants.GripperState getGripperState() {
        return scoringLevel.gripperState;
    }

    public ElevatorConstants.ElevatorState getElevatorState() {
        return scoringLevel.elevatorState;
    }

    private void configureBindings() {
        configureScoringLevelBindings();
        configureReefHIDBindings();
        configureFallbackBindings();
    }

    private void configureScoringLevelBindings() {
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_GRIPPER_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L1_GRIPPER));
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_CORAL_INTAKE_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L1_CORAL_INTAKE));
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L2_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L2));
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L3_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L3));
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L4_TRIGGER.onTrue(getSetTargetLevelCommand(() -> CoralPlacingCommands.ScoringLevel.L4));
        OperatorConstants.INCREMENT_TARGET_SCORING_LEVEL_TRIGGER.onTrue(getSetTargetLevelCommand(this::getIncrementedScoringLevel));
        OperatorConstants.DECREMENT_TARGET_SCORING_LEVEL_TRIGGER.onTrue(getSetTargetLevelCommand(this::getDecrementedScoringLevel));
    }

    private Command getSetTargetLevelCommand(Supplier<CoralPlacingCommands.ScoringLevel> scoringLevel) {
        return new InstantCommand(
                () -> this.scoringLevel = scoringLevel.get()
        ).ignoringDisable(true);
    }

    private void configureReefHIDBindings() {
        for (int i = 0; i < 12; i++)
            hid.button(i + 1).onTrue(getSetFaceFromIndexCommand(i));
    }

    private Command getSetFaceFromIndexCommand(int index) {
        return new InstantCommand(
                () -> setFaceFromIndex(index)
        ).ignoringDisable(true);
    }

    private void setFaceFromIndex(int index) {
        clockPosition = FieldConstants.ReefClockPosition.values()[index / 2];
        reefSide = FieldConstants.ReefSide.values()[index % 2];
    }

    private void configureFallbackBindings() {
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_2_OCLOCK_TRIGGER.onTrue(getSetTargetClockPositionCommand(FieldConstants.ReefClockPosition.REEF_2_OCLOCK));
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_4_OCLOCK_TRIGGER.onTrue(getSetTargetClockPositionCommand(FieldConstants.ReefClockPosition.REEF_4_OCLOCK));
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_6_OCLOCK_TRIGGER.onTrue(getSetTargetClockPositionCommand(FieldConstants.ReefClockPosition.REEF_6_OCLOCK));
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_8_OCLOCK_TRIGGER.onTrue(getSetTargetClockPositionCommand(FieldConstants.ReefClockPosition.REEF_8_OCLOCK));
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_10_OCLOCK_TRIGGER.onTrue(getSetTargetClockPositionCommand(FieldConstants.ReefClockPosition.REEF_10_OCLOCK));
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_12_OCLOCK_TRIGGER.onTrue(getSetTargetClockPositionCommand(FieldConstants.ReefClockPosition.REEF_12_OCLOCK));

        OperatorConstants.SET_TARGET_REEF_SCORING_SIDE_LEFT_TRIGGER.onTrue(getSetTargetReefSideCommand(FieldConstants.ReefSide.LEFT));
        OperatorConstants.SET_TARGET_REEF_SCORING_SIDE_RIGHT_TRIGGER.onTrue(getSetTargetReefSideCommand(FieldConstants.ReefSide.RIGHT));
    }

    private Command getSetTargetClockPositionCommand(FieldConstants.ReefClockPosition clockPosition) {
        return new InstantCommand(
                () -> this.clockPosition = clockPosition
        ).ignoringDisable(true);
    }

    private Command getSetTargetReefSideCommand(FieldConstants.ReefSide reefSide) {
        return new InstantCommand(
                () -> this.reefSide = reefSide
        ).ignoringDisable(true);
    }

    private CoralPlacingCommands.ScoringLevel getIncrementedScoringLevel() {
        return switch (scoringLevel) {
            case L2 -> CoralPlacingCommands.ScoringLevel.L3;
            case L1_CORAL_INTAKE -> CoralPlacingCommands.ScoringLevel.L2;
            default -> CoralPlacingCommands.ScoringLevel.L4;
        };
    }

    private CoralPlacingCommands.ScoringLevel getDecrementedScoringLevel() {
        return switch (scoringLevel) {
            case L4 -> CoralPlacingCommands.ScoringLevel.L3;
            case L3 -> CoralPlacingCommands.ScoringLevel.L2;
            default -> CoralPlacingCommands.ScoringLevel.L1_CORAL_INTAKE;
        };
    }
}
