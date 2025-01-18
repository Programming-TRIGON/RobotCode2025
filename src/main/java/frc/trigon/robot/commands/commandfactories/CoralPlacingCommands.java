package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.flippable.FlippablePose2d;

public class CoralPlacingCommands {
    public static boolean SHOULD_SCORE_AUTONOMOUSLY = true;
    public static ScoringLevel TARGET_SCORING_LEVEL = ScoringLevel.L4;
    public static FieldConstants.ReefClockPosition TARGET_SCORING_REEF_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_6_OCLOCK;
    public static FieldConstants.ReefSide TARGET_SCORING_REEF_SIDE = FieldConstants.ReefSide.LEFT;

    public static Command getScoreInReefCommand() {
        return new ConditionalCommand(
                getAutonomouslyScoreInReefCommand(),
                getManuallyScoreInReefCommand(),
                () -> SHOULD_SCORE_AUTONOMOUSLY
        );
    }

    private static Command getManuallyScoreInReefCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(() -> TARGET_SCORING_LEVEL.elevatorState),
                getGripperSequenceCommand()
        );
    }

    private static Command getAutonomouslyScoreInReefCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(() -> TARGET_SCORING_LEVEL.elevatorState),
                getAutonomousDriveToReefThenManualDriveCommand(),
                getGripperSequenceCommand()
        );
    }

    private static Command getAutonomousDriveToReefThenManualDriveCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        () -> TARGET_SCORING_LEVEL.calculateTargetPlacingPosition(TARGET_SCORING_REEF_CLOCK_POSITION, TARGET_SCORING_REEF_SIDE),
                        PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS
                ),
                GeneralCommands.duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND)
        );
    }

    private static Command getGripperSequenceCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateCommand(() -> TARGET_SCORING_LEVEL.gripperState).until(CoralPlacingCommands::canContinueScoring),
                GripperCommands.getScoreInReefCommand()
        );
    }

    private static boolean canContinueScoring() {
        return RobotContainer.ELEVATOR.atTargetState() &&
                RobotContainer.GRIPPER.atTargetAngle() &&
                OperatorConstants.CONTINUE_SCORING_TRIGGER.getAsBoolean();
    }

    public enum ScoringLevel {
        L1(0.3, 0.1),
        L2(0.3, 0.1),
        L3(0.3, 0.1),
        L4(0.3, 0.1);

        final double xTransform, positiveYTransform;
        final ElevatorConstants.ElevatorState elevatorState;
        final GripperConstants.GripperState gripperState;

        ScoringLevel(double xTransform, double positiveYTransform) {
            this.xTransform = xTransform;
            this.positiveYTransform = positiveYTransform;
            this.elevatorState = determineElevatorState();
            this.gripperState = determineGripperState();
        }

        public FlippablePose2d calculateTargetPlacingPosition(FieldConstants.ReefClockPosition reefClockPosition, FieldConstants.ReefSide reefSide) {
            final Pose2d reefCenterPose = new Pose2d(FieldConstants.REEF_CENTER_TRANSLATION, reefClockPosition.clockAngle);
            final double yTransform = reefSide.shouldFlipYTransform(reefClockPosition) ? -positiveYTransform : positiveYTransform;
            final Transform2d transform = new Transform2d(xTransform, yTransform, new Rotation2d());
            return new FlippablePose2d(reefCenterPose.plus(transform), reefClockPosition.isFacingDriverStation);
        }

        private ElevatorConstants.ElevatorState determineElevatorState() {
            return switch (this) {
                case L1 -> ElevatorConstants.ElevatorState.SCORE_L1;
                case L2 -> ElevatorConstants.ElevatorState.SCORE_L2;
                case L3 -> ElevatorConstants.ElevatorState.SCORE_L3;
                case L4 -> ElevatorConstants.ElevatorState.SCORE_L4;
            };
        }

        private GripperConstants.GripperState determineGripperState() {
            return switch (this) {
                case L1 -> GripperConstants.GripperState.PREPARE_L1;
                case L2, L3 -> GripperConstants.GripperState.PREPARE_L3_OR_L2;
                case L4 -> GripperConstants.GripperState.PREPARE_L4;
            };
        }
    }
}