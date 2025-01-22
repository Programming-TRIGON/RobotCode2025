package frc.trigon.robot.misc.objectdetectioncamera.io;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private final PhotonCamera photonCamera;

    public PhotonObjectDetectionCameraIO(String hostname) {
        PhotonCamera.setVersionCheckEnabled(false);
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        if (!photonCamera.isConnected())
            return;
        final PhotonPipelineResult result = getLatestPipelineResult();
        if (result == null) {
            updateNoNewResultInputs(inputs);
            return;
        }

        final List<PhotonTrackedTarget>[] visibleGamePieces = calculateVisibleObjects(result);

        boolean hasAnyTarget = false;
        for (int i = 0; i < ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES; i++) {
            inputs.hasTarget[i] = !visibleGamePieces[i].isEmpty();
            if (inputs.hasTarget[i])
                hasAnyTarget = true;
        }

        if (hasAnyTarget) {
            updateHasNewResultInputs(visibleGamePieces, inputs);
            return;
        }
        updateNoNewResultInputs(inputs);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateNoNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.hasTarget = new boolean[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        inputs.visibleObjectYaws = new Rotation2d[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES][0];
    }

    private void updateHasNewResultInputs(List<PhotonTrackedTarget>[] visibleGamePieces, ObjectDetectionCameraInputsAutoLogged inputs) {
        for (int i = 0; i < visibleGamePieces.length; i++)
            if (inputs.hasTarget[i])
                inputs.visibleObjectYaws[i] = getVisibleTargetObjectYaws(visibleGamePieces[i], i);
    }

    private List<PhotonTrackedTarget>[] calculateVisibleObjects(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget>[] visibleObjects = new List[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        for (int i = 0; i < visibleObjects.length; i++)
            visibleObjects[i] = calculateVisibleTargetObjects(result, i);
        return visibleObjects;
    }

    private List<PhotonTrackedTarget> calculateVisibleTargetObjects(PhotonPipelineResult result, int targetId) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final ArrayList<PhotonTrackedTarget> visibleTargetObjects = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            if (target.objDetectId == targetId)
                visibleTargetObjects.add(target);
        }
        return visibleTargetObjects;
    }

    private Rotation2d[] getVisibleTargetObjectYaws(List<PhotonTrackedTarget> visibleObjects, int targetId) {
        final Rotation2d[] visibleObjectYaws = new Rotation2d[visibleObjects.size()];
        visibleObjectYaws[0] = getClosestTargetObjectYaw(visibleObjects, targetId);

        boolean hasSeenBestTarget = false;
        for (int i = 0; i < visibleObjectYaws.length; i++) {
            final Rotation2d objectYaw = Rotation2d.fromDegrees(-visibleObjects.get(i).getYaw());
            if (objectYaw.equals(visibleObjectYaws[0])) {
                hasSeenBestTarget = true;
                continue;
            }
            visibleObjectYaws[hasSeenBestTarget ? i : i + 1] = objectYaw;
        }
        return visibleObjectYaws;
    }

    /**
     * Gets the yaw of the closest object to the camera.
     *
     * @param visibleObjects the objects visible to the camera
     * @param targetId       the ID of the target object
     * @return the closest object's yaw relative to the camera
     */
    private Rotation2d getClosestTargetObjectYaw(List<PhotonTrackedTarget> visibleObjects, int targetId) {
        double closestObjectDistance = Double.POSITIVE_INFINITY;
        Rotation2d bestObjectYaw = null;
        for (PhotonTrackedTarget object : visibleObjects) {
            final double currentObjectDistance = Math.abs(object.getYaw()) + Math.abs(object.getPitch());
            if (closestObjectDistance > currentObjectDistance &&
                    object.objDetectId == targetId) {
                closestObjectDistance = currentObjectDistance;
                bestObjectYaw = Rotation2d.fromDegrees(-object.getYaw());
            }
        }
        return bestObjectYaw;
    }
}
