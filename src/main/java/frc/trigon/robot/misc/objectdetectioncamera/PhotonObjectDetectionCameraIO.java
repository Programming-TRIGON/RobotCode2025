package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private final PhotonCamera photonCamera;
    private final Rotation2d cameraMountYaw;

    protected PhotonObjectDetectionCameraIO(String hostname, Rotation2d cameraMountYaw) {
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

        final List<PhotonTrackedTarget> visibleCoral = calculateVisibleTargetObjects(result, 0);
        final List<PhotonTrackedTarget> visibleAlgae = calculateVisibleTargetObjects(result, 1);

        inputs.hasCoralTarget = !visibleCoral.isEmpty();
        inputs.hasAlgaeTarget = !visibleAlgae.isEmpty();

        if (!inputs.hasCoralTarget && !inputs.hasAlgaeTarget) {
            updateNoNewResultInputs(inputs);
            return;
        }

        updateHasNewResultInputs(visibleCoral, visibleAlgae, inputs);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
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

    private void updateNoNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.visibleObjectYaws = new Rotation2d[2][0];
    }

    private void updateHasNewResultInputs(List<PhotonTrackedTarget> visibleCoral, List<PhotonTrackedTarget> visibleAlgae, ObjectDetectionCameraInputsAutoLogged inputs) {
        if (inputs.hasCoralTarget)
            inputs.visibleObjectYaws[SimulatedGamePieceConstants.GamePieceType.CORAL.id] = getTargetVisibleObjectYaws(visibleCoral, 0);
        if (inputs.hasAlgaeTarget)
            inputs.visibleObjectYaws[SimulatedGamePieceConstants.GamePieceType.ALGAE.id] = getTargetVisibleObjectYaws(visibleAlgae, 1);
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
