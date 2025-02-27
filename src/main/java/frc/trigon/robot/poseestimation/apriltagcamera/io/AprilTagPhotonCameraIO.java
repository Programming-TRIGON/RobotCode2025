package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    final PhotonCamera photonCamera;

    public AprilTagPhotonCameraIO(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = getLatestPipelineResult();

        inputs.hasResult = latestResult != null && latestResult.hasTargets();
        if (inputs.hasResult) {
            updateHasResultInputs(inputs, latestResult);
            return;
        }

        updateNoResultInputs(inputs);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateHasResultInputs(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        updateSolvePNPPoses(inputs, latestResult);
        if (inputs.bestCameraSolvePNPPose == null) {
            updateNoResultInputs(inputs);
            inputs.hasResult = false;
            return;
        }

        inputs.latestResultTimestampSeconds = latestResult.getTimestampSeconds();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
        inputs.poseAmbiguity = latestResult.getMultiTagResult().isPresent() ? 0 : latestResult.getBestTarget().getPoseAmbiguity();
        inputs.distancesFromTags = getDistancesFromTags(latestResult);
    }

    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.bestCameraSolvePNPPose = new Pose3d();
        inputs.alternateCameraSolvePNPPose = new Pose3d();
        inputs.visibleTagIDs = new int[0];
    }

    private void updateSolvePNPPoses(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        if (latestResult.getMultiTagResult().isPresent()) {
            final Transform3d cameraPoseTransform = latestResult.getMultiTagResult().get().estimatedPose.best;
            inputs.bestCameraSolvePNPPose = new Pose3d().plus(cameraPoseTransform).relativeTo(FieldConstants.APRIL_TAG_FIELD_LAYOUT.getOrigin());
            inputs.alternateCameraSolvePNPPose = inputs.bestCameraSolvePNPPose;
            return;
        }

        final Pose3d tagPose = FieldConstants.TAG_ID_TO_POSE.get(latestResult.getBestTarget().getFiducialId());
        if (tagPose == null) {
            inputs.bestCameraSolvePNPPose = null;
            return;
        }

        final Transform3d bestTargetToCamera = latestResult.getBestTarget().getBestCameraToTarget().inverse();
        final Transform3d alternateTargetToCamera = latestResult.getBestTarget().getBestCameraToTarget().inverse();

        inputs.bestCameraSolvePNPPose = tagPose.transformBy(bestTargetToCamera);
        inputs.alternateCameraSolvePNPPose = tagPose.transformBy(alternateTargetToCamera);
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final int[] visibleTagIDs = new int[targets.size()];

        for (int i = 0; i < targets.size(); i++)
            visibleTagIDs[i] = targets.get(i).getFiducialId();

        return visibleTagIDs;
    }

    private double[] getDistancesFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final double[] distances = new double[targets.size()];

        for (int i = 0; i < targets.size(); i++)
            distances[i] = getDistanceFromTarget(targets.get(i));

        return distances;
    }

    private double getDistanceFromTarget(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }
}
