package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.CoralAutoDriveCommand;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import org.json.simple.parser.ParseException;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableRotation2d;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    private static final BooleanEvent SWITCH_TO_CORAL_FEEDBACK = new BooleanEvent(CommandScheduler.getInstance().getActiveButtonLoop(), () -> CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition() != null).rising();
    private static final BooleanEvent SWITCH_TO_PP_FEEDBACK = new BooleanEvent(CommandScheduler.getInstance().getActiveButtonLoop(), () -> CameraConstants.OBJECT_DETECTION_CAMERA.getTrackedObjectFieldRelativePosition() != null).falling();

    public static Command getScoreInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return CoralCollectionCommands.getLoadCoralCommand().andThen(
                new ParallelCommandGroup(
                        ElevatorCommands.getSetTargetStateCommand(() -> scoringLevel.elevatorState),
                        GripperCommands.getPrepareForStateCommand(() -> scoringLevel.gripperState)
                )
        );
    }

    public static Command getScoreInReefFromGripperUntilReachedCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return CoralCollectionCommands.getLoadCoralCommand().andThen(
                new ParallelCommandGroup(
                        ElevatorCommands.getSetTargetStateCommand(() -> scoringLevel.elevatorState),
                        GripperCommands.getPrepareForStateCommand(() -> scoringLevel.gripperState)
                ).until(() -> RobotContainer.ELEVATOR.atState(scoringLevel.elevatorState) && RobotContainer.GRIPPER.atState(scoringLevel.gripperState))
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
            throw new RuntimeException(e);
        }
    }
}