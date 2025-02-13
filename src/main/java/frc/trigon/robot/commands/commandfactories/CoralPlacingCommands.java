package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.WaitUntilChangeCommand;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.flippable.FlippablePose2d;

public class CoralPlacingCommands {
    public static boolean SHOULD_SCORE_AUTONOMOUSLY = false;
    public static ScoringLevel TARGET_SCORING_LEVEL = ScoringLevel.L4;
    public static FieldConstants.ReefClockPosition TARGET_REEF_SCORING_CLOCK_POSITION = FieldConstants.ReefClockPosition.REEF_6_OCLOCK;
    public static FieldConstants.ReefSide TARGET_REEF_SCORING_SIDE = FieldConstants.ReefSide.RIGHT;

    public static Command getScoreInReefCommand() {
        return new ConditionalCommand(
                getScoreInReefFromCoralIntakeCommand(),
                getScoreInReefFromGripperCommand(),
                () -> TARGET_SCORING_LEVEL == ScoringLevel.L1_CORAL_INTAKE
        ).alongWith(
                getWaitUntilScoringTargetChangesCommand().andThen(
                        () -> getScoreInReefCommand().onlyWhile(OperatorConstants.SCORE_CORAL_IN_REEF_TRIGGER).schedule()
                )
        );
    }

    private static Command getWaitUntilScoringTargetChangesCommand() {
        return new ParallelRaceGroup(
                new WaitUntilChangeCommand<>(() -> TARGET_SCORING_LEVEL),
                new WaitUntilChangeCommand<>(() -> TARGET_REEF_SCORING_CLOCK_POSITION),
                new WaitUntilChangeCommand<>(() -> TARGET_REEF_SCORING_SIDE)
        );
    }

    private static Command getScoreInReefFromCoralIntakeCommand() {
        return new ConditionalCommand(
                getAutonomouslyScoreInReefFromCoralIntakeCommand().asProxy(),
                getCoralIntakeScoringSequnceCommand().asProxy(),
                () -> SHOULD_SCORE_AUTONOMOUSLY
        );
    }

    private static Command getScoreInReefFromGripperCommand() {
        return new ConditionalCommand(
                getAutonomouslyScoreInReefFromGripperCommand().asProxy(),
                getManuallyScoreInReefFromGripperCommand().asProxy(),
                () -> SHOULD_SCORE_AUTONOMOUSLY
        ).finallyDo(
                (interrupted) -> {
                    if (interrupted && TARGET_SCORING_LEVEL.ordinal() > ScoringLevel.L2.ordinal() && RobotContainer.ELEVATOR.willMovementMoveThroughHitRange(0))
                        getMakeSureGripperDoesntHitReefCommand().schedule();
                }
        );
    }

    private static Command getAutonomouslyScoreInReefFromCoralIntakeCommand() {
        return new ParallelCommandGroup(
                getAutonomousDriveToReefThenManualDriveCommand(),
                getCoralIntakeScoringSequnceCommand()
        );
    }

    private static Command getCoralIntakeScoringSequnceCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getUnloadCoralCommand(),
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1).until(CoralPlacingCommands::canContinueScoringFromCoralIntake),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1)
        );
    }

    private static Command getManuallyScoreInReefFromGripperCommand() {
        return CoralCollectionCommands.getLoadCoralCommand().andThen(
                new ParallelCommandGroup(
                        ElevatorCommands.getSetTargetStateCommand(() -> TARGET_SCORING_LEVEL.elevatorState),
                        getGripperScoringSequenceCommand()
                )
        );
    }

    private static Command getAutonomouslyScoreInReefFromGripperCommand() {
        return new ParallelCommandGroup(
                CoralCollectionCommands.getLoadCoralCommand().andThen(
                        new ParallelCommandGroup(
                                getOpenElevatorWhenCloseToReefCommand(),
                                getGripperScoringSequenceCommand()
                        )
                ),
                getAutonomousDriveToReefThenManualDriveCommand()
        );
    }

    private static Command getMakeSureGripperDoesntHitReefCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(() -> GripperConstants.GripperState.AFTER_ELEVATOR_OPEN_POSITION),
                Commands.idle(RobotContainer.ELEVATOR)
        ).until(() -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.AFTER_ELEVATOR_OPEN_POSITION));
    }

    private static Command getGripperScoringSequenceCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getPrepareForStateCommand(() -> TARGET_SCORING_LEVEL.gripperState).until(CoralPlacingCommands::canContinueScoringFromGripper),
                GripperCommands.getSetTargetStateCommand(() -> TARGET_SCORING_LEVEL.gripperState)
        );
    }

    private static Command getOpenElevatorWhenCloseToReefCommand() {
        return GeneralCommands.runWhen(
                ElevatorCommands.getSetTargetStateCommand(() -> TARGET_SCORING_LEVEL.elevatorState),
                () -> calculateDistanceToTargetScoringPose() < PathPlannerConstants.MINIMUM_DISTANCE_FROM_REEF_TO_OPEN_ELEVATOR
        );
    }

    private static Command getAutonomousDriveToReefThenManualDriveCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        CoralPlacingCommands::calculateTargetScoringPose,
                        PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS
                ),
                GeneralCommands.getFieldRelativeDriveCommand()
        );
    }

    private static double calculateDistanceToTargetScoringPose() {
        final Translation2d currentTranslation = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getTranslation();
        final Translation2d targetTranslation = calculateTargetScoringPose().get().getTranslation();
        return currentTranslation.getDistance(targetTranslation);
    }

    public static FlippablePose2d calculateTargetScoringPose() {
        return TARGET_SCORING_LEVEL.calculateTargetPlacingPosition(TARGET_REEF_SCORING_CLOCK_POSITION, TARGET_REEF_SCORING_SIDE);
    }

    private static boolean canContinueScoringFromCoralIntake() {
        return RobotContainer.CORAL_INTAKE.atTargetAngle() &&
                OperatorConstants.CONTINUE_SCORING_TRIGGER.getAsBoolean();
    }

    private static boolean canContinueScoringFromGripper() {
        return RobotContainer.ELEVATOR.atTargetState() &&
                RobotContainer.GRIPPER.atTargetAngle() &&
                OperatorConstants.CONTINUE_SCORING_TRIGGER.getAsBoolean();
//                RobotContainer.SWERVE.atPose(calculateTargetScoringPose());
    }

    /**
     * An enum that represents the different levels of scoring in the reef.
     * Each level has a different x and y transform from the reef center,
     * as well as a different elevator and gripper state.
     * The x and y transform are used to calculate the target placing position from the middle of the reef.
     */
    public enum ScoringLevel {
        L1_CORAL_INTAKE(1.38, 0.14, Rotation2d.fromDegrees(180)),
        L1_GRIPPER(1.38, 0.17, Rotation2d.fromDegrees(0)),
        L2(1.3, 0.14, Rotation2d.fromDegrees(0)),
        L3(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0)),
        L4(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0));

        public final int level = calculateLevel();
        final double xTransformMeters, positiveYTransformMeters;
        final Rotation2d rotationTransform;
        final ElevatorConstants.ElevatorState elevatorState;
        final GripperConstants.GripperState gripperState;

        /**
         * Constructs a scoring level with the given x and y transform.
         * The elevator and gripper state are determined automatically based on the scoring level.
         *
         * @param xTransformMeters         the x transform from the middle of the reef to the target placing position
         * @param positiveYTransformMeters the y transform from the middle of the reef to the target placing position.
         *                                 This must be positive (to account for flipping later on), and might be flipped depending on operator input (left or right reef side)
         * @param rotationTransform        the angle to be facing the reef with the robot. Might change when scoring from the coral intake
         */
        ScoringLevel(double xTransformMeters, double positiveYTransformMeters, Rotation2d rotationTransform) {
            this.xTransformMeters = xTransformMeters;
            this.positiveYTransformMeters = positiveYTransformMeters;
            this.rotationTransform = rotationTransform;
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
            final Transform2d transform = new Transform2d(xTransformMeters, yTransform, rotationTransform);

            return new FlippablePose2d(reefCenterPose.plus(transform), true);
        }

        private ElevatorConstants.ElevatorState determineElevatorState() {
            return switch (ordinal()) {
                case 0 -> null;
                case 1 -> ElevatorConstants.ElevatorState.SCORE_L1;
                case 2 -> ElevatorConstants.ElevatorState.SCORE_L2;
                case 3 -> ElevatorConstants.ElevatorState.SCORE_L3;
                case 4 -> ElevatorConstants.ElevatorState.SCORE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private GripperConstants.GripperState determineGripperState() {
            return switch (ordinal()) {
                case 0 -> null;
                case 1 -> GripperConstants.GripperState.SCORE_L1;
                case 2, 3 -> GripperConstants.GripperState.SCORE_L3_OR_L2;
                case 4 -> GripperConstants.GripperState.SCORE_L4;
                default -> throw new IllegalStateException("Unexpected value: " + ordinal());
            };
        }

        private int calculateLevel() {
            if (ordinal() == 0)
                return 1;
            return ordinal();
        }
    }
}