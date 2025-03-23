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
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.trigon.utilities.flippable.FlippablePose2d;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    public static final LoggedNetworkBoolean[] SCORED_L4S = getEmptyL4LoggedNetworkBooleanArray();
    private static FlippablePose2d TARGET_SCORING_POSE = null;

    public static Command getFloorAutonomousCommand(boolean isRight) {
        return getFloorAutonomousCommand(isRight, FieldConstants.ReefClockPosition.values());
    }

    public static Command getFloorAutonomousCommand(boolean isRight, FieldConstants.ReefClockPosition[] reefClockPositions) {
        return getCycleCoralCommand(isRight, reefClockPositions).repeatedly().withName("FloorAutonomous" + (isRight ? "Right" : "Left") + (reefClockPositions.length * 2) + "Branches");
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
                new SequentialCommandGroup(
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.EJECT_UPWARDS).until(() -> RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.REST)),
                        GripperCommands.getGripperDefaultCommand()
                ),
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
                ).withTimeout(3)
        ).repeatedly();
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

    public static Command getDriveToReefAndScoreCommand(FieldConstants.ReefClockPosition[] reefClockPositions) {
        return new ParallelCommandGroup(
                getDriveToReefWithoutHittingAlgaeCommand(reefClockPositions),
                getCoralSequenceCommand()
        );
    }

    public static Command getDriveToReefWithoutHittingAlgaeCommand(FieldConstants.ReefClockPosition[] reefClockPositions) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> TARGET_SCORING_POSE = calculateClosestScoringPose(true, reefClockPositions, false)),
                new WaitUntilCommand(() -> TARGET_SCORING_POSE != null).raceWith(SwerveCommands.getClosedLoopSelfRelativeDriveCommand(() -> 0, () -> 0, () -> 0)),
                SwerveCommands.getDriveToPoseCommand(() -> calculateClosestScoringPose(true, reefClockPositions, true), PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS).repeatedly().until(RobotContainer.ELEVATOR::isElevatorOverAlgaeHitRange),
                SwerveCommands.getDriveToPoseCommand(() -> TARGET_SCORING_POSE, PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS).repeatedly().until(AutonomousCommands::canFeed)
        );
    }

    public static Command getCoralSequenceCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getLoadCoralCommand(),
                new WaitUntilCommand(() -> TARGET_SCORING_POSE != null),
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
                getAddCurrentScoringBranchToScoredBranchesCommand()
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

    public static FlippablePose2d calculateClosestScoringPose(boolean shouldOnlyCheckOpenBranches, FieldConstants.ReefClockPosition[] reefClockPositions, boolean shouldStayBehindAlgae) {
        final boolean[] scoredBranchesAtL4 = getScoredBranchesAtL4();
        final Pose2d currentRobotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();

        double closestDistance = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = null;
        for (FieldConstants.ReefClockPosition currentClockPosition : reefClockPositions) {
            for (FieldConstants.ReefSide currentSide : FieldConstants.ReefSide.values()) {
                if (shouldOnlyCheckOpenBranches && scoredBranchesAtL4[currentClockPosition.ordinal() * 2 + currentSide.ordinal()])
                    continue;
                final Pose2d reefSideScoringPose = CoralPlacingCommands.ScoringLevel.L4.calculateTargetPlacingPosition(currentClockPosition, currentSide).get();
                final double distance = currentRobotPose.getTranslation().getDistance(reefSideScoringPose.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    if (shouldStayBehindAlgae)
                        closestScoringPose = reefSideScoringPose.transformBy(new Transform2d(new Translation2d(0.1, 0), new Rotation2d()));
                    else
                        closestScoringPose = reefSideScoringPose;
                }
            }
        }

        return closestScoringPose == null ? null : new FlippablePose2d(closestScoringPose, false);
    }

    @AutoLogOutput
    private static boolean canFeed() {
        return RobotContainer.ELEVATOR.atState(OperatorConstants.REEF_CHOOSER.getScoringLevel().elevatorState) &&
                RobotContainer.GRIPPER.atState(OperatorConstants.REEF_CHOOSER.getScoringLevel().gripperState) &&
                TARGET_SCORING_POSE != null &&
                RobotContainer.SWERVE.atPose(TARGET_SCORING_POSE);
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

    private static boolean[] getScoredBranchesAtL4() {
        final boolean[] booleanArray = new boolean[SCORED_L4S.length];

        for (int i = 0; i < booleanArray.length; i++)
            booleanArray[i] = SCORED_L4S[i].get();

        return booleanArray;
    }

    private static Command getAddCurrentScoringBranchToScoredBranchesCommand() {
        return new InstantCommand(
                () -> {
                    final int branchNumber = getBranchNumberFromScoringPose(TARGET_SCORING_POSE.get());
                    SCORED_L4S[branchNumber].set(true);
                }
        );
    }

    private static int getBranchNumberFromScoringPose(Pose2d scoringPose) {
        for (FieldConstants.ReefClockPosition currentClockPosition : FieldConstants.ReefClockPosition.values()) {
            for (FieldConstants.ReefSide currentSide : FieldConstants.ReefSide.values()) {
                final Pose2d reefSideScoringPose = CoralPlacingCommands.ScoringLevel.L4.calculateTargetPlacingPosition(currentClockPosition, currentSide).get();
                if (reefSideScoringPose.getTranslation().getDistance(scoringPose.getTranslation()) < 0.01)
                    return currentClockPosition.ordinal() * 2 + currentSide.ordinal();
            }
        }

        return 0;
    }

    private static LoggedNetworkBoolean[] getEmptyL4LoggedNetworkBooleanArray() {
        final LoggedNetworkBoolean[] array = new LoggedNetworkBoolean[12];
        for (int i = 0; i < array.length; i++)
            array[i] = new LoggedNetworkBoolean("ScoredL4s/" + i, false);
        return array;
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