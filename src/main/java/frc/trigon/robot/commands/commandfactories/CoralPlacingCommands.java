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
import frc.trigon.robot.misc.ReefChooser;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableTranslation2d;

public class CoralPlacingCommands {
    public static boolean SHOULD_SCORE_AUTONOMOUSLY = true;
    private static final ReefChooser REEF_CHOOSER = OperatorConstants.REEF_CHOOSER;

    public static Command getScoreInReefCommand() {
        return new ConditionalCommand(
                getCoralIntakeScoringSequenceCommand().asProxy(),
                getScoreInReefFromGripperCommand().asProxy(),
                () -> REEF_CHOOSER.getScoringLevel() == ScoringLevel.L1_CORAL_INTAKE
        ).raceWith(getWaitUntilScoringTargetChangesCommand()).andThen(
                () -> getScoreInReefCommand().onlyWhile(OperatorConstants.SCORE_CORAL_IN_REEF_TRIGGER).schedule()
        );
    }

    private static Command getWaitUntilScoringTargetChangesCommand() {
        return new ParallelRaceGroup(
                new WaitUntilChangeCommand<>(REEF_CHOOSER::getScoringLevel),
                new WaitUntilChangeCommand<>(REEF_CHOOSER::getClockPosition),
                new WaitUntilChangeCommand<>(REEF_CHOOSER::getReefSide),
                new WaitUntilChangeCommand<>(OperatorConstants.OVERRIDE_AUTO_SCORING_TO_CLOSEST_SCORING_LOCATION_TRIGGER::getAsBoolean)
        );
    }

    private static Command getScoreInReefFromGripperCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getAutonomouslyScoreInReefFromGripperCommand().asProxy(),
                getManuallyScoreInReefFromGripperCommand().asProxy(),
                () -> SHOULD_SCORE_AUTONOMOUSLY && !OperatorConstants.MULTIFUNCTION_TRIGGER.getAsBoolean() && REEF_CHOOSER.getScoringLevel() != ScoringLevel.L1_GRIPPER
        );
    }

    private static Command getCoralIntakeScoringSequenceCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getUnloadCoralCommand(),
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1).until(CoralPlacingCommands::canContinueScoringFromCoralIntake),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1_BOOST).withTimeout(0.5),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.SCORE_L1)
        );
    }

    private static Command getManuallyScoreInReefFromGripperCommand() {
        return CoralCollectionCommands.getLoadCoralCommand().andThen(
                new ParallelCommandGroup(
                        ElevatorCommands.getSetTargetStateCommand(REEF_CHOOSER::getElevatorState),
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

    private static Command getGripperScoringSequenceCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF)
                        .unless(() -> RobotContainer.ELEVATOR.atState(REEF_CHOOSER.getElevatorState()) || REEF_CHOOSER.getScoringLevel() == ScoringLevel.L2 || REEF_CHOOSER.getScoringLevel() == ScoringLevel.L1_GRIPPER)
                        .until(() -> RobotContainer.ELEVATOR.atState(REEF_CHOOSER.getElevatorState())),
                GripperCommands.getPrepareForStateCommand(REEF_CHOOSER::getGripperState).until(CoralPlacingCommands::canContinueScoringFromGripper),
                GripperCommands.getSetTargetStateCommand(REEF_CHOOSER::getGripperState)
        );
    }

    private static Command getOpenElevatorWhenCloseToReefCommand() {
        return GeneralCommands.runWhen(
                ElevatorCommands.getSetTargetStateCommand(REEF_CHOOSER::getElevatorState),
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

    public static FlippablePose2d calculateTargetScoringPose() {
        if (OperatorConstants.OVERRIDE_AUTO_SCORING_TO_CLOSEST_SCORING_LOCATION_TRIGGER.getAsBoolean() || true)
            return calculateClosestScoringPose();
        return REEF_CHOOSER.calculateTargetScoringPose();
    }

    private static double calculateDistanceToTargetScoringPose() {
        final Translation2d currentTranslation = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d targetTranslation = REEF_CHOOSER.calculateTargetScoringPose().get().getTranslation();
        return currentTranslation.getDistance(targetTranslation);
    }

    private static FlippablePose2d calculateClosestScoringPose() {
        final Translation2d robotPositionOnField = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Pose2d closestReefSide = calculateClosestReefSide(robotPositionOnField);

        return new FlippablePose2d(
                calculateClosestScoringPositionFromReefSide(closestReefSide, robotPositionOnField),
                closestReefSide.getRotation().getRadians(),
                false
        );
    }

    /**
     * Calculates the closest reef side to the robot and returns the position at which the robot is aligned directly to the center of that side.
     * This position can then be transformed to score on the desired branch (left or right).
     *
     * @param robotPositionOnField the position of the robot on the field
     * @return the position where the robot is aligned with the closest reef side.
     */
    private static Pose2d calculateClosestReefSide(Translation2d robotPositionOnField) {
        final Transform2d REEF_CENTER_TO_REEF_CLOCK_ANGLE_SCORING_POSITION_TRANSFORM = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d());
        final Translation2d reefCenterPositionOnField = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();

        double closestReefClockPositionDistanceMeters = Double.POSITIVE_INFINITY;
        Rotation2d closestReefClockPositionAngle = new Rotation2d();

        for (Rotation2d reefClockAngle : FieldConstants.REEF_CLOCK_ANGLES) {
            final Pose2d reefCenterPoseAtAngle = new Pose2d(reefCenterPositionOnField, reefClockAngle);
            final Translation2d scoringPositionOnField = reefCenterPoseAtAngle.transformBy(REEF_CENTER_TO_REEF_CLOCK_ANGLE_SCORING_POSITION_TRANSFORM).getTranslation();
            final double distanceToScoringPositionOnFieldMeters = scoringPositionOnField.getDistance(robotPositionOnField);
            if (distanceToScoringPositionOnFieldMeters < closestReefClockPositionDistanceMeters) {
                closestReefClockPositionDistanceMeters = distanceToScoringPositionOnFieldMeters;
                closestReefClockPositionAngle = reefClockAngle;
            }
        }

        return new Pose2d(reefCenterPositionOnField, closestReefClockPositionAngle).transformBy(REEF_CENTER_TO_REEF_CLOCK_ANGLE_SCORING_POSITION_TRANSFORM);
    }

    /**
     * Calculates the closest scoring position to the robot.
     *
     * @param reefSideScoringPosition the target scoring side, where the robot is centered between both branches
     * @param robotPositionOnField    the position of the robot on the field
     * @return the closest scoring position to the robot
     */
    private static Translation2d calculateClosestScoringPositionFromReefSide(Pose2d reefSideScoringPosition, Translation2d robotPositionOnField) {
        final Transform2d reefSideToRightScoringPositionTransform = new Transform2d(new Translation2d(0, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS), new Rotation2d());
        final Translation2d
                closestRightBranchScoringPositionOnField = reefSideScoringPosition.transformBy(reefSideToRightScoringPositionTransform).getTranslation(),
                closestLeftBranchScoringPositionOnField = reefSideScoringPosition.transformBy(reefSideToRightScoringPositionTransform.inverse()).getTranslation();

        final double
                closestRightBranchScoringPositionDistanceMeters = closestRightBranchScoringPositionOnField.getDistance(robotPositionOnField),
                closestLeftBranchScoringPositionDistanceMeters = closestLeftBranchScoringPositionOnField.getDistance(robotPositionOnField);

        final boolean isClosestScoringPositionLeft = closestRightBranchScoringPositionDistanceMeters > closestLeftBranchScoringPositionDistanceMeters;
        return isClosestScoringPositionLeft ? closestLeftBranchScoringPositionOnField : closestRightBranchScoringPositionOnField;
    }

    private static boolean canContinueScoringFromCoralIntake() {
        return RobotContainer.CORAL_INTAKE.atTargetAngle() &&
                OperatorConstants.CONTINUE_TRIGGER.getAsBoolean();
    }

    private static boolean canContinueScoringFromGripper() {
        return RobotContainer.ELEVATOR.atTargetState() &&
                RobotContainer.GRIPPER.atTargetAngle() &&
                OperatorConstants.CONTINUE_TRIGGER.getAsBoolean();
//                RobotContainer.SWERVE.atPose(calculateTargetScoringPose());
    }

    /**
     * An enum that represents the different levels of scoring in the reef.
     * Each level has a different x and y transform from the reef center,
     * as well as a different elevator and gripper state.
     * The x and y transform are used to calculate the target placing position from the middle of the reef.
     */
    public enum ScoringLevel {
        L1_CORAL_INTAKE(Double.NaN, Double.NaN, null),
        L1_GRIPPER(Double.NaN, Double.NaN, null),
        L2(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, Rotation2d.fromDegrees(0)),
        L3(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0)),
        L4(L2.xTransformMeters, L2.positiveYTransformMeters, Rotation2d.fromDegrees(0));

        public final ElevatorConstants.ElevatorState elevatorState;
        public final GripperConstants.GripperState gripperState;
        public final int level = calculateLevel();
        final double xTransformMeters, positiveYTransformMeters;
        final Rotation2d rotationTransform;

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
                case 0 -> ElevatorConstants.ElevatorState.REST;
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