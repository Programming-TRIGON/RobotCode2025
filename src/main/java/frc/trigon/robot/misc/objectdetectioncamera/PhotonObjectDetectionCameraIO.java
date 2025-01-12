package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private final PhotonCamera photonCamera;

    protected PhotonObjectDetectionCameraIO(String hostname) {
        PhotonCamera.setVersionCheckEnabled(false);
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        if (!photonCamera.isConnected())
            return;
        final PhotonPipelineResult result = getLatestPipelineResult();
        if (result == null) {
            updateNoResultInputs(inputs);
            return;
        }

        inputs.hasCoralTarget = hasTarget(0, result);
        inputs.hasAlgaeTarget = hasTarget(1, result);

        if (!inputs.hasCoralTarget && !inputs.hasAlgaeTarget) {
            updateNoResultInputs(inputs);
            return;
        }

        updateHasResultInputs(result, inputs);
    }

    private void updateNoResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.hasCoralTarget = false;
        inputs.hasAlgaeTarget = false;
        inputs.visibleCoralYaws = new Rotation2d[0];
        inputs.visibleAlgaeYaws = new Rotation2d[0];
    }

    private void updateHasResultInputs(PhotonPipelineResult result, ObjectDetectionCameraInputsAutoLogged inputs) {
        if (inputs.hasCoralTarget)
            inputs.visibleCoralYaws = getTargetVisibleObjectYaws(result, 0);
        if (inputs.hasAlgaeTarget)
            inputs.visibleAlgaeYaws = getTargetVisibleObjectYaws(result, 1);
    }

    private boolean hasTarget(int targetId, PhotonPipelineResult result) {
        for (int i = 0; i < result.getTargets().size(); i++)
            if (result.getTargets().get(i).objDetectId == targetId)
                return true;
        return false;
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private Rotation2d[] getTargetVisibleObjectYaws(PhotonPipelineResult result, int targetId) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final Rotation2d[] visibleObjectYaws = new Rotation2d[targets.size()];
        visibleObjectYaws[0] = getBestTargetYaw(result, targetId);

        boolean hasSeenBestTarget = false;
        for (int i = 0; i < visibleObjectYaws.length; i++) {
            final Rotation2d targetYaw = Rotation2d.fromDegrees(-targets.get(i).getYaw());
            if (targetYaw.equals(visibleObjectYaws[0])) {
                hasSeenBestTarget = true;
                continue;
            }
            visibleObjectYaws[hasSeenBestTarget ? i : i + 1] = targetYaw;
        }
        return visibleObjectYaws;
    }

    private Rotation2d getBestTargetYaw(PhotonPipelineResult result, int targetId) {
        double closestTargetDistance = Double.POSITIVE_INFINITY;
        Rotation2d bestTarget = null;
        for (PhotonTrackedTarget target : result.getTargets()) {
            final double currentTargetDistance = Math.abs(target.getYaw()) + Math.abs(target.getPitch());
            if (closestTargetDistance > currentTargetDistance) {
                closestTargetDistance = currentTargetDistance;
                bestTarget = Rotation2d.fromDegrees(-target.getYaw());
            }
        }
        return bestTarget;
    }
}
