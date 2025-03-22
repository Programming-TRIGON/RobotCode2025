package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.CoralAutoDriveCommand;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableTranslation2d;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    public static Command getFloorAutonomousCommand(boolean isRight) {
        return getCycleCoralCommand(isRight).repeatedly().withName("FloorAutonomous" + (isRight ? "Right" : "Left"));
    }

    public static Command getFloorAutonomousCommand(boolean isRight, FieldConstants.ReefClockPosition[] reefClockPositions) {
        return getCycleCoralCommand(isRight, reefClockPositions).repeatedly().withName("FloorAutonomous" + (isRight ? "Right" : "Left") + "4Branches");
    }

    public static Command getCycleCoralCommand(boolean isRight) {
        return new SequentialCommandGroup(
                getDriveToReefAndScoreCommand(),
                getCollectCoralCommand(isRight)
        );
    }

    public static Command getCycleCoralCommand(boolean isRight, FieldConstants.ReefClockPosition[] reefClockPositions) {
        return new SequentialCommandGroup(
                getDriveToReefAndScoreCommand(reefClockPositions),
                getCollectCoralCommand(isRight)
        );
    }

    public static Command getCollectCoralCommand(boolean isRight) {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                GripperCommands.getGripperDefaultCommand(),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR),
                getDriveToCoralCommand(isRight)
        )
                .until(() -> RobotContainer.CORAL_INTAKE.isEarlyCoralCollectionDetected() || RobotContainer.CORAL_INTAKE.hasGamePiece())
                .unless(() -> RobotContainer.CORAL_INTAKE.hasGamePiece() || RobotContainer.GRIPPER.hasGamePiece());
    }

    public static Command getDriveToCoralCommand(boolean isRight) {
        return new SequentialCommandGroup(
                new InstantCommand(CameraConstants.OBJECT_DETECTION_CAMERA::initializeTracking),
                getFindCoralCommand(isRight).unless(() -> CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition() != null).until(() -> CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition() != null),
                new ParallelCommandGroup(
                        CoralAutoDriveCommand.getDriveToCoralCommand(CoralAutoDriveCommand::calculateDistanceFromTrackedCoral),
                        new RunCommand(() -> {
                            CameraConstants.OBJECT_DETECTION_CAMERA.trackObject(SimulatedGamePieceConstants.GamePieceType.CORAL);
                            Logger.recordOutput("Distance", CoralAutoDriveCommand.calculateDistanceFromTrackedCoral());
                        })
                )
        );
    }

    public static Command getFindCoralCommand(boolean isRight) {
        return new ParallelCommandGroup(
                new RunCommand(() -> CameraConstants.OBJECT_DETECTION_CAMERA.trackObject(SimulatedGamePieceConstants.GamePieceType.CORAL)),
                SwerveCommands.getDriveToPoseCommand(() -> isRight ? FieldConstants.AUTO_FIND_CORAL_POSE_RIGHT : FieldConstants.AUTO_FIND_CORAL_POSE_LEFT, new PathConstraints(2.5, 4, Units.degreesToRadians(900), Units.degreesToRadians(1200))).andThen(
                        SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                                () -> 0,
                                () -> 0,
                                () -> 0.2
                        )
                )
        );
    }

    public static Command getDriveToReefAndScoreCommand() {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(() -> calculateClosestScoringPose(true), PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS).repeatedly().until(AutonomousCommands::canFeed),
                getCoralSequenceCommand()
        );
    }

    public static Command getDriveToReefAndScoreCommand(FieldConstants.ReefClockPosition[] reefClockPositions) {
        return new ParallelCommandGroup(
                SwerveCommands.getDriveToPoseCommand(() -> calculateClosestScoringPose(true, reefClockPositions), PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS).repeatedly().until(AutonomousCommands::canFeed),
                getCoralSequenceCommand()
        );
    }

    public static Command getCoralSequenceCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getLoadCoralCommand(),
                getScoreCommand()
        );
    }

    public static Command getScoreCommand() {
        return new SequentialCommandGroup(
                getPrepareForScoreCommand().until(AutonomousCommands::canFeed),
                getFeedCoralCommand()
        ).raceWith(
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR)
        );
    }

    public static Command getPrepareForScoreCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(OperatorConstants.REEF_CHOOSER.getElevatorState()),
                GripperCommands.getPrepareForStateCommand(OperatorConstants.REEF_CHOOSER.getScoringLevel().gripperState)
        );
    }

    public static Command getFeedCoralCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(OperatorConstants.REEF_CHOOSER.getElevatorState()),
                GripperCommands.getSetTargetStateCommand(OperatorConstants.REEF_CHOOSER.getGripperState()),
                CoralPlacingCommands.getAddCurrentScoringBranchToScoredBranchesCommand()
        ).withTimeout(0.25);
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

    public static Command getPrepareForScoringInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return CoralCollectionCommands.getLoadCoralCommand().andThen(
                new ParallelCommandGroup(
                        CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.REST),
                        ElevatorCommands.getSetTargetStateCommand(() -> scoringLevel.elevatorState),
                        getGripperScoringSequenceCommand(scoringLevel)
                )
        );
    }

    public static FlippablePose2d calculateClosestScoringPose(boolean shouldOnlyCheckOpenBranches) {
        final FieldConstants.ReefClockPosition[] reefClockPositions = FieldConstants.ReefClockPosition.values();
        final Translation2d robotPositionOnField = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Transform2d
                reefCenterToRightBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS + (RobotContainer.ELEVATOR.isElevatorOverAlgaeHitRange() ? 0 : 0.05), FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d()),
                reefCenterToLeftBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS + (RobotContainer.ELEVATOR.isElevatorOverAlgaeHitRange() ? 0 : 0.05), -FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d());

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (int i = 0; i < reefClockPositions.length; i++) {
            final FieldConstants.ReefClockPosition targetReefPosition = reefClockPositions[i];
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, targetReefPosition.clockAngle);
            for (int j = 0; j < 2; j++) {
                if (CoralPlacingCommands.getScoredBranchesAtCurrentLevel()[targetReefPosition.ordinal() * 2 + j] && shouldOnlyCheckOpenBranches)
                    continue;

                final Pose2d currentScoringPose = reefCenterAtTargetRotation.transformBy(j == 0 ? reefCenterToRightBranchScoringPose : reefCenterToLeftBranchScoringPose);
                final double distanceFromCurrentScoringPoseMeters = currentScoringPose.getTranslation().getDistance(robotPositionOnField);
                if (distanceFromCurrentScoringPoseMeters < distanceFromClosestScoringPoseMeters) {
                    distanceFromClosestScoringPoseMeters = distanceFromCurrentScoringPoseMeters;
                    closestScoringPose = currentScoringPose;
                }
            }
        }

        return new FlippablePose2d(closestScoringPose, false);
    }

    public static FlippablePose2d calculateClosestScoringPose(boolean shouldOnlyCheckOpenBranches, FieldConstants.ReefClockPosition[] reefClockPositions) {
        final Translation2d robotPositionOnField = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Transform2d
                reefCenterToRightBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS + (RobotContainer.ELEVATOR.isElevatorOverAlgaeHitRange() ? 0 : 0.05), FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d()),
                reefCenterToLeftBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS + (RobotContainer.ELEVATOR.isElevatorOverAlgaeHitRange() ? 0 : 0.05), -FieldConstants.REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS, new Rotation2d());

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (int i = 0; i < reefClockPositions.length; i++) {
            final FieldConstants.ReefClockPosition targetReefPosition = reefClockPositions[i];
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, targetReefPosition.clockAngle);
            for (int j = 0; j < 2; j++) {
                if (CoralPlacingCommands.getScoredBranchesAtCurrentLevel()[targetReefPosition.ordinal() * 2 + j] && shouldOnlyCheckOpenBranches)
                    continue;

                final Pose2d currentScoringPose = reefCenterAtTargetRotation.transformBy(j == 0 ? reefCenterToRightBranchScoringPose : reefCenterToLeftBranchScoringPose);
                final double distanceFromCurrentScoringPoseMeters = currentScoringPose.getTranslation().getDistance(robotPositionOnField);
                if (distanceFromCurrentScoringPoseMeters < distanceFromClosestScoringPoseMeters) {
                    distanceFromClosestScoringPoseMeters = distanceFromCurrentScoringPoseMeters;
                    closestScoringPose = currentScoringPose;
                }
            }
        }

        return new FlippablePose2d(closestScoringPose, false);
    }

    @AutoLogOutput
    private static boolean canFeed() {
        return RobotContainer.ELEVATOR.atState(OperatorConstants.REEF_CHOOSER.getScoringLevel().elevatorState) &&
                RobotContainer.GRIPPER.atState(OperatorConstants.REEF_CHOOSER.getScoringLevel().gripperState) &&
                RobotContainer.SWERVE.atPose(calculateClosestScoringPose(false));
    }

    private static Command getGripperScoringSequenceCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF)
                        .unless(() -> RobotContainer.ELEVATOR.atState(scoringLevel.elevatorState))
                        .until(() -> RobotContainer.ELEVATOR.atState(scoringLevel.elevatorState)),
                GripperCommands.getPrepareForStateCommand(scoringLevel.gripperState)
        );
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