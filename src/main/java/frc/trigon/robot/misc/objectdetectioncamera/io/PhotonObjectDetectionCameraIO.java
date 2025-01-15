package frc.trigon.robot.misc.objectdetectioncamera.io;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private static final int NUMBER_OF_GAME_PIECE_TYPES = SimulatedGamePieceConstants.GamePieceType.values().length;
    private final PhotonCamera photonCamera;
    private final Rotation2d cameraMountYaw;

    public PhotonObjectDetectionCameraIO(String hostname, Rotation2d cameraMountYaw) {
        PhotonCamera.setVersionCheckEnabled(false);
        photonCamera = new PhotonCamera(hostname);
        this.cameraMountYaw = cameraMountYaw;
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

        final List<PhotonTrackedTarget>[] visibleGamePieces = calculateVisibleGamePieces(result);

        boolean hasAnyTarget = false;
        for (int i = 0; i < NUMBER_OF_GAME_PIECE_TYPES; i++) {
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
        inputs.hasTarget = new boolean[NUMBER_OF_GAME_PIECE_TYPES];
        inputs.visibleObjectYaws = new Rotation2d[NUMBER_OF_GAME_PIECE_TYPES][0];
    }

    private List<PhotonTrackedTarget>[] calculateVisibleGamePieces(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget>[] visibleGamePieces = new List[SimulatedGamePieceConstants.GamePieceType.values().length];
        for (int i = 0; i < SimulatedGamePieceConstants.GamePieceType.values().length; i++)
            visibleGamePieces[i] = calculateVisibleTargetObjects(result, i);
        return visibleGamePieces;
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

    private void updateHasNewResultInputs(List<PhotonTrackedTarget>[] visibleGamePieces, ObjectDetectionCameraInputsAutoLogged inputs) {
        for (int i = 0; i < visibleGamePieces.length; i++)
            if (inputs.hasTarget[i])
                inputs.visibleObjectYaws[i] = getTargetVisibleObjectYaws(visibleGamePieces[i], i);
    }

    private Rotation2d[] getTargetVisibleObjectYaws(List<PhotonTrackedTarget> visibleObjects, int targetId) {
        final Rotation2d[] visibleObjectYaws = new Rotation2d[visibleObjects.size()];
        visibleObjectYaws[0] = getBestTargetYaw(visibleObjects, targetId);

        boolean hasSeenBestTarget = false;
        for (int i = 0; i < visibleObjectYaws.length; i++) {
            final Rotation2d targetYaw = Rotation2d.fromDegrees(-visibleObjects.get(i).getYaw()).plus(cameraMountYaw);
            if (targetYaw.equals(visibleObjectYaws[0])) {
                hasSeenBestTarget = true;
                continue;
            }
            visibleObjectYaws[hasSeenBestTarget ? i : i + 1] = targetYaw;
        }
        return visibleObjectYaws;
    }

    private Rotation2d getBestTargetYaw(List<PhotonTrackedTarget> visibleObjects, int targetId) {
        double closestTargetDistance = Double.POSITIVE_INFINITY;
        Rotation2d bestTarget = null;
        for (PhotonTrackedTarget target : visibleObjects) {
            final double currentTargetDistance = Math.abs(target.getYaw()) + Math.abs(target.getPitch());
            if (closestTargetDistance > currentTargetDistance
                    && target.objDetectId == targetId) {
                closestTargetDistance = currentTargetDistance;
                bestTarget = Rotation2d.fromDegrees(-target.getYaw()).plus(cameraMountYaw);
            }
        }
        return bestTarget;
    }
}
