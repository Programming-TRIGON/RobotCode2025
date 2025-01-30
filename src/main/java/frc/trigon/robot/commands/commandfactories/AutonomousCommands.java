package frc.trigon.robot.commands.commandfactories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.json.simple.parser.ParseException;
import org.trigon.utilities.flippable.FlippablePose2d;

import java.io.IOException;
import java.util.function.Supplier;

/**
 * A class that contains command factories for preparation commands and commands used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    public static Command getScoreInReefFromElevatorCommand(CoralPlacingCommands.ScoringLevel scoringLevel) {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(() -> scoringLevel.elevatorState),
                GripperCommands.getPrepareForStateCommand(() -> scoringLevel.gripperState)
        );
    }

    public static Command getAlignToCoralCommand() {
        return new FunctionalCommand(
                () -> {
//                    PPHolonomicDriveController.overrideYFeedback(() -> -CoralAlignmentCommand.Y_PID_CONTROLLER.calculate(CameraConstants.CORAL_DETECTION_CAMERA.getTrackedObjectRotation().getDegrees()));
//                    PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose().getRotation().plus(CameraConstants.CORAL_DETECTION_CAMERA.getTrackedObjectRotation())));
                },
                () -> CameraConstants.OBJECT_DETECTION_CAMERA.trackObject(SimulatedGamePieceConstants.GamePieceType.CORAL),
                (interrupted) -> {
//                    PPHolonomicDriveController.clearFeedbackOverrides();
//                    PPHolonomicDriveController.setRotationTargetOverride(Optional::empty);
                },
                () -> RobotContainer.CORAL_INTAKE.hasGamePiece() || RobotContainer.GRIPPER.hasGamePiece()
        );
    }

    public static Command getCollectCoralCommand() {
        return new ParallelCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST),
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.REST)
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