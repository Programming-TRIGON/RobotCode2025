package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

import java.util.List;
import java.util.Optional;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    private final Transform3d robotToCamera;
    final PhotonCamera photonCamera;

    public AprilTagPhotonCameraIO(String cameraName, Transform3d robotToCamera) {
        photonCamera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
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

    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.bestCameraSolvePNPPose = new Pose3d();
        inputs.alternateCameraSolvePNPPose = inputs.bestCameraSolvePNPPose;
        inputs.constrainedSolvePNPPose = inputs.alternateCameraSolvePNPPose;
        inputs.visibleTagIDs = new int[0];
        inputs.hasConstrainedResult = false;
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
        inputs.hasConstrainedResult = false;
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

//        updateConstrainedSolvePNPPose(inputs, latestResult);
    }

    private void updateConstrainedSolvePNPPose(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final Pose3d constrainedSolvePNPPose = calculateConstrainedSolvePNPPose(latestResult, inputs.bestCameraSolvePNPPose);
        if (constrainedSolvePNPPose == null) {
            inputs.hasConstrainedResult = false;
            return;
        }

        inputs.constrainedSolvePNPPose = constrainedSolvePNPPose;
        inputs.hasConstrainedResult = true;
    }

    private Pose3d calculateConstrainedSolvePNPPose(PhotonPipelineResult result, Pose3d bestCameraSolvePNPPose) {
        final Optional<Matrix<N3, N3>> cameraMatrix = photonCamera.getCameraMatrix();
        final Optional<Matrix<N8, N1>> distCoeffs = photonCamera.getDistCoeffs();

        if (cameraMatrix.isEmpty() || distCoeffs.isEmpty())
            return null;

        Pose3d fieldToRobotSeed = bestCameraSolvePNPPose.transformBy(this.robotToCamera.inverse());
        final Rotation2d robotYawAtTimestamp = RobotContainer.POSE_ESTIMATOR.getEstimatedPoseAtTimestamp(result.getTimestampSeconds()).getRotation();

        if (!AprilTagCameraConstants.CONSTRAINED_SOLVE_PNP_PARAMS.headingFree()) {
            fieldToRobotSeed = new Pose3d(
                    fieldToRobotSeed.getTranslation(),
                    new Rotation3d(robotYawAtTimestamp)
            );
        }

        final Optional<PnpResult> pnpResult = VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                cameraMatrix.get(),
                distCoeffs.get(),
                result.getTargets(),
                robotToCamera,
                fieldToRobotSeed,
                FieldConstants.APRIL_TAG_FIELD_LAYOUT,
                TargetModel.kAprilTag36h11,
                AprilTagCameraConstants.CONSTRAINED_SOLVE_PNP_PARAMS.headingFree(),
                robotYawAtTimestamp,
                AprilTagCameraConstants.CONSTRAINED_SOLVE_PNP_PARAMS.headingScaleFactor()
        );

        return pnpResult.map(value -> Pose3d.kZero.plus(value.best).transformBy(robotToCamera)).orElse(null);
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
