package frc.trigon.robot.commands.commandclasses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.flippable.FlippableRotation2d;

public class CoralAlignmentCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.OBJECT_DETECTION_CAMERA;
    private FlippableRotation2d prev = null;

    public CoralAlignmentCommand() {
        addCommands(
                new InstantCommand(() -> prev = null),
                getDriveWhileAligningToNoteCommand(),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR)
        );
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> 0,
                () -> 0,
                () -> {
                    var a = CAMERA.getTargetObjectsRotations(SimulatedGamePieceConstants.GamePieceType.CORAL);
                    if (a.length == 0)
                        return prev;
                    prev = new FlippableRotation2d(RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getRotation().plus(a[0].toRotation2d()), false);
                    return prev;
                }
        );
    }

    private boolean shouldAlignToNote() {
        return CAMERA.hasTargets(SimulatedGamePieceConstants.GamePieceType.CORAL) && !RobotContainer.GRIPPER.hasGamePiece();
    }
}