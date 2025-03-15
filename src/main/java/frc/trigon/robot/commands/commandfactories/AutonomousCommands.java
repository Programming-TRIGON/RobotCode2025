package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.CoralAutoDriveCommand;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableRotation2d;
import org.trigon.utilities.flippable.FlippableTranslation2d;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    private static final BooleanEvent SWITCH_TO_CORAL_FEEDBACK = new BooleanEvent(CommandScheduler.getInstance().getActiveButtonLoop(), () -> CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition() != null).rising();
    private static final BooleanEvent SWITCH_TO_PP_FEEDBACK = new BooleanEvent(CommandScheduler.getInstance().getActiveButtonLoop(), () -> CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition() != null).falling();
    private static final boolean[] BRANCHES_SCORED = new boolean[12];

    public static Command getPOCCommand() {
        return new SequentialCommandGroup(
                getDriveToReefAndScoreCommand(),
                getCollectCoralCommand(),
                getDriveToReefAndScoreCommand(),
                getCollectCoralCommand(),
                getDriveToReefAndScoreCommand(),
                getCollectCoralCommand(),
                getDriveToReefAndScoreCommand()
        );
    }

    public static Command getDriveUntilCoralIsVisibleCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(() -> new FlippablePose2d(3.5, 2, Rotation2d.fromDegrees(180), true), PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS));
//        ).until(() -> CAMERA.getTrackedObjectFieldRelativePosition() != null);
    }

    public static Command getCollectCoralCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.REST),
                getDriveUntilCoralIsVisibleCommand().andThen(
                        new CoralAutoDriveCommand()
                ),
                getIntakeUntilHasCoralCommand()
        ).until(RobotContainer.CORAL_INTAKE::hasGamePiece);
    }

    public static Command getDriveToReefAndScoreCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(AutonomousCommands::calculateClosestScoringPose, PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS),
                getCoralSequenceCommand()
        );
    }

    public static Command getIntakeUntilHasCoralCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR)
                        .until(RobotContainer.CORAL_INTAKE::isEarlyCoralCollectionDetected),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.CENTER_CORAL)
        ).until(RobotContainer.CORAL_INTAKE::hasGamePiece);
    }

    public static Command getCoralSequenceCommand() {
        return new SequentialCommandGroup(
                getLoadCoralCommand(),
                getScoreCommand()
        );
    }

    public static Command getLoadCoralCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.LOAD_CORAL),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK)//.onlyWhile(() -> RobotContainer.GRIPPER.atTargetAngle() && RobotContainer.ELEVATOR.atTargetState())
        ).until(RobotContainer.GRIPPER::hasGamePiece);
    }

    public static Command getScoreCommand() {
        return new SequentialCommandGroup(
                getPrepareForScoreCommand().until(() ->
                        RobotContainer.ELEVATOR.atTargetState() &&
                                RobotContainer.GRIPPER.atTargetAngle() &&
                                RobotContainer.SWERVE.atPose(calculateClosestScoringPose())),
                getFeedCoralCommand()
        );
    }

    public static Command getPrepareForScoreCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_L4),
                GripperCommands.getPrepareForStateCommand(GripperConstants.GripperState.SCORE_L4)
        );
    }

    public static Command getFeedCoralCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_L4),
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_L4),
                new InstantCommand(() -> BRANCHES_SCORED[getBranchNumberFromScoringPose(calculateClosestScoringPose().get())] = true)
        ).withTimeout(0.5);
    }

    public static FlippablePose2d calculateClosestScoringPose() {
        final Translation2d robotPositionOnField = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d
                reefCenterToRightBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d()),
                reefCenterToLeftBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, -FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d());

        final Pose2d[] scoringPoses = new Pose2d[12];
        for (int i = 0; i < reefClockAngles.length; i++) {
            final Rotation2d targetRotation = reefClockAngles[i];
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, targetRotation);
            for (int j = 0; j < 2; j++) {
                scoringPoses[i * 2 + j] = reefCenterAtTargetRotation.transformBy(j == 0 ? reefCenterToRightBranchScoringPose : reefCenterToLeftBranchScoringPose);
            }
        }

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (int i = 0; i < scoringPoses.length; i++) {
            if (BRANCHES_SCORED[i]) {
                continue;
            }
            final double distance = scoringPoses[i].getTranslation().getDistance(robotPositionOnField);
            if (distance < distanceFromClosestScoringPoseMeters) {
                distanceFromClosestScoringPoseMeters = distance;
                closestScoringPose = scoringPoses[i];
            }
        }

        return new FlippablePose2d(closestScoringPose, false);
    }

    public static int getBranchNumberFromScoringPose(Pose2d scoringPose) {
        final Translation2d reefCenterTranslation = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;

        for (int i = 0; i < reefClockAngles.length; i++) {
            if (reefClockAngles[i].equals(scoringPose.getRotation())) {
                final Transform2d reefSideToRightScoringPositionTransform = new Transform2d(new Translation2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS, FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS), new Rotation2d());
                final Pose2d reefSideScoringPosition = new Pose2d(reefCenterTranslation, reefClockAngles[i]);
                final Pose2d rightBranchScoringPositionOnField = reefSideScoringPosition.transformBy(reefSideToRightScoringPositionTransform);

                if (scoringPose.equals(rightBranchScoringPositionOnField))
                    return i * 2;
                else
                    return i * 2 + 1;
            }
        }
        return 0;
    }

    public static Command getPrepareForScoringInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return CoralCollectionCommands.getLoadCoralCommand().andThen(
                new ParallelCommandGroup(
                        CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.REST),
                        ElevatorCommands.getSetTargetStateCommand(() -> scoringLevel.elevatorState),
                        getGripperScoringSequenceCommand(scoringLevel)
                )
        );
    }

    public static Command getScoreInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return new SequentialCommandGroup(
                getPrepareForScoringInReefFromGripperCommand(scoringLevel)
                        .unless(() -> RobotContainer.ELEVATOR.atState(scoringLevel.elevatorState) && RobotContainer.GRIPPER.atState(scoringLevel.gripperState))
                        .until(() -> RobotContainer.ELEVATOR.atState(scoringLevel.elevatorState) && RobotContainer.GRIPPER.atState(scoringLevel.gripperState)),
                new WaitCommand(0.1),
                GripperCommands.getSetTargetStateCommand(scoringLevel.gripperState).withTimeout(0.6)
        );
    }

    private static Command getGripperScoringSequenceCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF)
                        .unless(() -> RobotContainer.ELEVATOR.atState(scoringLevel.elevatorState))
                        .until(() -> RobotContainer.ELEVATOR.atState(scoringLevel.elevatorState)),
                GripperCommands.getPrepareForStateCommand(scoringLevel.gripperState)
        );
    }

    public static final double P = 0.6;

    public static Command getAlignToCoralCommand() {
        return new FunctionalCommand(
                () -> {
                    CameraConstants.OBJECT_DETECTION_CAMERA.initializeTracking();
                    PPHolonomicDriveController.setRotationTargetOverride(() -> {
                        final FlippableRotation2d targetAngle = CoralAutoDriveCommand.calculateTargetAngle();
                        if (targetAngle == null)
                            return Optional.empty();

                        return Optional.of(targetAngle.get());
                    });
                },
                () -> {
                    CameraConstants.OBJECT_DETECTION_CAMERA.trackObject(SimulatedGamePieceConstants.GamePieceType.CORAL);
                    if (SWITCH_TO_CORAL_FEEDBACK.getAsBoolean())
                        switchToCoralFeedback();
                    else if (SWITCH_TO_PP_FEEDBACK.getAsBoolean())
                        PPHolonomicDriveController.clearFeedbackOverrides();
                },
                (interrupted) -> {
                    PPHolonomicDriveController.clearFeedbackOverrides();
                    PPHolonomicDriveController.setRotationTargetOverride(Optional::empty);
                },
                () -> RobotContainer.CORAL_INTAKE.hasGamePiece() || RobotContainer.GRIPPER.hasGamePiece()
        );
    }

    private static void switchToCoralFeedback() {
        PPHolonomicDriveController.overrideXYFeedback(AutonomousCommands::calculateXFeedback, AutonomousCommands::calculateYFeedback);
    }

    private static double calculateXFeedback() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d trackedObjectPositionOnField = CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectPositionOnField == null)
            return 0;

        final Translation2d difference = trackedObjectPositionOnField.minus(robotPose.getTranslation());
        return difference.getX() * P;
    }

    private static double calculateYFeedback() {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d trackedObjectPositionOnField = CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition();
        if (trackedObjectPositionOnField == null)
            return 0;

        final Translation2d difference = trackedObjectPositionOnField.minus(robotPose.getTranslation());
        return difference.getY() * P;
    }

    public static Command getCollectCoralFromFloorCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                GripperCommands.getGripperDefaultCommand(),
                getAlignToCoralCommand()
        ).until(() -> RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece());
    }

    public static Command getCollectCoralFromFeederCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FEEDER),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                GripperCommands.getGripperDefaultCommand()
        ).until(() -> RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece());
    }

    /**
     * Creates a command that resets the pose estimator's pose to the starting pose of the given autonomous as long as the robot is not enabled.
     *
     * @param autoName the name of the autonomous
     * @return a command that resets the robot's pose estimator pose to the start position of the given autonomous
     */
    public static Command getResetPoseToAutoPoseCommand(Supplier<String> autoName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    RobotContainer.POSE_ESTIMATOR.resetPose(getAutoStartPose(autoName.get()));
                }
        ).ignoringDisable(true);
    }

    /**
     * Gets the starting position of the target PathPlanner autonomous.
     *
     * @param autoName the name of the autonomous group
     * @return the staring pose of the autonomous
     */
    public static Pose2d getAutoStartPose(String autoName) {
        try {
            final Pose2d nonFlippedAutoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
            final FlippablePose2d flippedAutoStartPose = new FlippablePose2d(nonFlippedAutoStartPose, true);
            return flippedAutoStartPose.get();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            return new Pose2d();
        }
    }
}