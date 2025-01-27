package frc.trigon.robot.misc.objectdetectioncamera.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraInputsAutoLogged;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_VISIBLE_DISTANCE_METERS = 5,
            MINIMUM_VISIBLE_DISTANCE_METERS = 0.05,
            MAXIMUM_VISIBLE_HEIGHT_METERS = 0.6;

    private final String hostname;
    private final Rotation2d cameraMountYaw;

    public SimulationObjectDetectionCameraIO(String hostname, Rotation2d cameraMountYaw) {
        this.hostname = hostname;
        this.cameraMountYaw = cameraMountYaw;
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final ArrayList<SimulatedGamePiece>[] visibleGamePieces = calculateAllVisibleGamePieces(robotPose);

        boolean hasAnyTarget = false;
        for (int i = 0; i < ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES; i++) {
            inputs.hasTarget[i] = !visibleGamePieces[i].isEmpty();
            if (inputs.hasTarget[i])
                hasAnyTarget = true;
        }

        if (hasAnyTarget) {
            updateHasNewResultInputs(visibleGamePieces, robotPose, inputs);
            return;
        }

        updateNoNewResultInputs(inputs);
    }

    private ArrayList<SimulatedGamePiece>[] calculateAllVisibleGamePieces(Pose2d robotPose) {
        final ArrayList<SimulatedGamePiece>[] visibleGamePieces = new ArrayList[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        for (int i = 0; i < visibleGamePieces.length; i++)
            visibleGamePieces[i] = calculateVisibleGamePieces(robotPose, i);
        return visibleGamePieces;
    }

    private void updateNoNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.hasTarget = new boolean[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        inputs.visibleObjectYaws = new Rotation2d[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES][0];
    }

    private void updateHasNewResultInputs(ArrayList<SimulatedGamePiece>[] visibleGamePieces, Pose2d robotPose, ObjectDetectionCameraInputsAutoLogged inputs) {
        for (int i = 0; i < visibleGamePieces.length; i++) {
            if (inputs.hasTarget[i]) {
                final SimulatedGamePiece closestGamePiece = calculateClosestVisibleObject(robotPose, visibleGamePieces[i]);
                inputs.visibleObjectYaws[i] = new Rotation2d[]{calculateCameraYawToObject(closestGamePiece.getPose(), robotPose)};
            }
        }

        logVisibleGamePieces(visibleGamePieces);
    }

    /**
     * Calculates the placements of all visible objects by checking if they are within range and within the horizontal FOV.
     *
     * @param robotPose the position of the robot on the field
     * @return the placements of the visible objects
     */
    private ArrayList<SimulatedGamePiece> calculateVisibleGamePieces(Pose2d robotPose, int targetId) {
        final ArrayList<SimulatedGamePiece> gamePiecesOnField = targetId == 0 ? SimulationFieldHandler.getSimulatedCoral() : SimulationFieldHandler.getSimulatedAlgae();
        final ArrayList<SimulatedGamePiece> visibleTargetObjects = new ArrayList<>();

        for (SimulatedGamePiece currentObject : gamePiecesOnField) {
            final Rotation2d cameraYawToObject = calculateCameraYawToObject(currentObject.getPose(), robotPose);
            if (isWithinHorizontalFOV(cameraYawToObject) &&
                    isWithinDistance(currentObject, robotPose))
                visibleTargetObjects.add(currentObject);
        }
        return visibleTargetObjects;
    }

    /**
     * Calculates the yaw of the closest visible target game piece relative to the camera by calculating the yaw of all target objects visible to the camera and returning the one with the smallest yaw deviation from the camera's center.
     *
     * @param robotPose the pose of the robot on the field
     * @return the yaw of the closest visible target object
     */
    private SimulatedGamePiece calculateClosestVisibleObject(Pose2d robotPose, ArrayList<SimulatedGamePiece> visibleGamePieces) {
        double closestObjectDistance = Double.POSITIVE_INFINITY;
        SimulatedGamePiece closestVisibleObject = null;

        for (SimulatedGamePiece object : visibleGamePieces) {
            final double robotDistanceToObject = object.getDistanceFromPoseMeters(new Pose3d(robotPose));
            if (robotDistanceToObject < closestObjectDistance) {
                closestObjectDistance = robotDistanceToObject;
                closestVisibleObject = object;
            }
        }

        return closestVisibleObject;
    }

    /**
     * Calculates the difference in yaw from an object to the camera's yaw.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return the yaw of the object relative to the camera
     */
    private Rotation2d calculateCameraYawToObject(Pose3d objectPlacement, Pose2d robotPose) {
        final Translation2d difference = objectPlacement.toPose2d().getTranslation().minus(robotPose.getTranslation());
        final Rotation2d differenceAngle = difference.getAngle();
        final Rotation2d cameraFieldRelativeAngle = robotPose.getRotation().plus(cameraMountYaw);
        return differenceAngle.minus(cameraFieldRelativeAngle);
    }

    /**
     * Checks if an object is within the field-of-view of the camera.
     *
     * @param objectYaw the yaw of the object relative to the camera
     * @return if the object is within the field-of-view of the camera
     */
    private boolean isWithinHorizontalFOV(Rotation2d objectYaw) {
        return Math.abs(objectYaw.getRadians()) <= CAMERA_HORIZONTAL_FOV.getRadians() / 2;
    }

    /**
     * Checks if an object is within the distance range of the camera.
     * The distance range is set at the top of this class as {@link #MINIMUM_VISIBLE_DISTANCE_METERS} and {@link #MINIMUM_VISIBLE_DISTANCE_METERS}.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return if the object is withing the distance range of the camera
     */
    private boolean isWithinDistance(SimulatedGamePiece objectPlacement, Pose2d robotPose) {
        final double robotToObjectDistanceMeters = objectPlacement.getDistanceFromPoseMeters(new Pose3d(robotPose));
        return objectPlacement.getPose().getZ() < MAXIMUM_VISIBLE_HEIGHT_METERS &&
                robotToObjectDistanceMeters <= MAXIMUM_VISIBLE_DISTANCE_METERS &&
                robotToObjectDistanceMeters >= MINIMUM_VISIBLE_DISTANCE_METERS;
    }

    private void logVisibleGamePieces(ArrayList<SimulatedGamePiece>[] visibleGamePieces) {
        for (int i = 0; i < visibleGamePieces.length; i++) {
            final String gamePieceTypeName = SimulatedGamePieceConstants.GamePieceType.getNameFromID(i);
            Logger.recordOutput(hostname + "/Visible" + gamePieceTypeName, mapSimulatedGamePieceListToPoseArray(visibleGamePieces[i]));
        }
    }

    private Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<SimulatedGamePiece> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = gamePieces.get(i).getPose();
        }
        return poses;
    }
}