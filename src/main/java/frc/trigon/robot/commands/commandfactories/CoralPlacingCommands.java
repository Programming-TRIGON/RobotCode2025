package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.WaitUntilChangeCommand;
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
    public static FieldConstants.ReefClockPosition TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_6_OCLOCK;
    public static FieldConstants.ReefSide TARGET_REEF_SCORING_SIDE = FieldConstants.ReefSide.LEFT;

    public static Command getScoreInReefCommand() {
        return new ConditionalCommand(
                getAutonomouslyScoreInReefCommand().asProxy(),
                getManuallyScoreInReefCommand().asProxy(),
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
                getOpenElevatorWhenCloseToReefCommand(),
                getAutonomousDriveToReefThenManualDriveCommand(),
                getGripperSequenceCommand(),
                getWaitUntilScoringTargetChangesCommand().andThen(
                        () -> getAutonomouslyScoreInReefCommand().onlyWhile(OperatorConstants.SCORE_CORAL_IN_REEF_TRIGGER).schedule()
                )
        );
    }

    private static Command getOpenElevatorWhenCloseToReefCommand() {
        return GeneralCommands.runWhen(
                ElevatorCommands.getSetTargetStateCommand(() -> TARGET_SCORING_LEVEL.elevatorState),
                () -> calculateDistanceToTargetScoringPose() < PathPlannerConstants.MINIMUM_DISTANCE_FROM_REEF_TO_OPEN_ELEVATOR
        );
    }

    private static Command getWaitUntilScoringTargetChangesCommand() {
        return new WaitUntilChangeCommand<>(() -> TARGET_SCORING_LEVEL)
                .raceWith(new WaitUntilChangeCommand<>(() -> TARGET_REEF_SCORING_CLOCK_POSITION))
                .raceWith(new WaitUntilChangeCommand<>(() -> TARGET_REEF_SCORING_SIDE));
    }

    private static Command getAutonomousDriveToReefThenManualDriveCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        CoralPlacingCommands::calculateTargetScoringPose,
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

    private static double calculateDistanceToTargetScoringPose() {
        final Translation2d currentTranslation = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getTranslation();
        final Translation2d targetTranslation = calculateTargetScoringPose().get().getTranslation();
        return currentTranslation.getDistance(targetTranslation);
    }

    private static FlippablePose2d calculateTargetScoringPose() {
        return TARGET_SCORING_LEVEL.calculateTargetPlacingPosition(TARGET_REEF_SCORING_CLOCK_POSITION, TARGET_REEF_SCORING_SIDE);
    }

    private static boolean canContinueScoring() {
        return RobotContainer.ELEVATOR.atTargetState() &&
                RobotContainer.GRIPPER.atTargetAngle() &&
                OperatorConstants.CONTINUE_SCORING_TRIGGER.getAsBoolean();
    }

    /**
     * An enum that represents the different levels of scoring in the reef.
     * Each level has a different x and y transform from the reef center,
     * as well as a different elevator and gripper state.
     * The x and y transform are used to calculate the target placing position from the middle of the reef.
     */
    public enum ScoringLevel {
        L1(1.3, 0.17),
        L2(1.3, 0.17),
        L3(1.3, 0.17),
        L4(1.3, 0.17);

        final double xTransformMeters, positiveYTransformMeters;
        final ElevatorConstants.ElevatorState elevatorState;
        final GripperConstants.GripperState gripperState;

        /**
         * Constructs a scoring level with the given x and y transform.
         * The elevator and gripper state are determined automatically based on the scoring level.
         *
         * @param xTransformMeters         the x transform from the middle of the reef to the target placing position
         * @param positiveYTransformMeters the y transform from the middle of the reef to the target placing position.
         *                                 This must be positive (to account for flipping later on), and might be flipped depending on operator input (left or right reef side)
         */
        ScoringLevel(double xTransformMeters, double positiveYTransformMeters) {
            this.xTransformMeters = xTransformMeters;
            this.positiveYTransformMeters = positiveYTransformMeters;
            this.elevatorState = determineElevatorState();
            this.gripperState = determineGripperState();
        }

        /**
         * Calculates the target placing position using the clock position and the target reef side.
         * The reef side transform will be flipped depending on operator input.
         * To make it more intuitive for the operator to input the reef side,
         * left will always correspond to the physical left side in the driver station,
         * as opposed to "reef relative" left.
         *
         * @param reefClockPosition the desired clock position of the reef
         * @param reefSide          the desired side of the reef, left or right (as seen from the driver station)
         * @return the target placing position
         */
        public FlippablePose2d calculateTargetPlacingPosition(FieldConstants.ReefClockPosition reefClockPosition, FieldConstants.ReefSide reefSide) {
            final Pose2d reefCenterPose = new Pose2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, reefClockPosition.clockAngle);
            final double yTransform = reefSide.shouldFlipYTransform(reefClockPosition) ? -positiveYTransformMeters : positiveYTransformMeters;
            final Transform2d transform = new Transform2d(xTransformMeters, yTransform, new Rotation2d());
            return new FlippablePose2d(reefCenterPose.plus(transform), true);
        }

        private ElevatorConstants.ElevatorState determineElevatorState() {
            return switch (ordinal()) {
                case 0 -> ElevatorConstants.ElevatorState.SCORE_L1;
                case 1 -> ElevatorConstants.ElevatorState.SCORE_L2;
                case 2 -> ElevatorConstants.ElevatorState.SCORE_L3;
                case 3 -> ElevatorConstants.ElevatorState.SCORE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private GripperConstants.GripperState determineGripperState() {
            return switch (ordinal()) {
                case 0 -> GripperConstants.GripperState.PREPARE_L1;
                case 1, 2 -> GripperConstants.GripperState.PREPARE_L3_OR_L2;
                case 3 -> GripperConstants.GripperState.PREPARE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }
    }
}