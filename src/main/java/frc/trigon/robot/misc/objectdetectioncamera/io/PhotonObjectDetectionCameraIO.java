package frc.trigon.robot.misc.objectdetectioncamera.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCameraInputsAutoLogged;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private final PhotonCamera photonCamera;

    public PhotonObjectDetectionCameraIO(String hostname) {
        PhotonCamera.setVersionCheckEnabled(false);
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        if (!photonCamera.isConnected()) {
            updateNoNewResultInputs(inputs);
            return;
        }

        final PhotonPipelineResult result = getLatestPipelineResult();
        if (result == null || !result.hasTargets()) {
            updateNoNewResultInputs(inputs);
            return;
        }

        updateHasNewResultInputs(inputs, result);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateNoNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.hasTarget = new boolean[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        inputs.visibleObjectRotations = new Rotation3d[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES][0];
    }

    private void updateHasNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs, PhotonPipelineResult result) {
        final List<Rotation3d>[] visibleObjectsRotations = new List[ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES];
        for (int i = 0; i < ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES; i++)
            visibleObjectsRotations[i] = new ArrayList<>();
        Arrays.fill(inputs.hasTarget, false);

        for (PhotonTrackedTarget currentTarget : result.getTargets()) {
            if (currentTarget.getDetectedObjectClassID() == -1)
                continue;

            inputs.hasTarget[currentTarget.getDetectedObjectClassID()] = true;
            visibleObjectsRotations[currentTarget.getDetectedObjectClassID()].add(extractRotation3d(currentTarget));
        }

        for (int i = 0; i < ObjectDetectionCameraConstants.NUMBER_OF_GAME_PIECE_TYPES; i++)
            inputs.visibleObjectRotations[i] = toArray(visibleObjectsRotations[i]);
    }

    private Rotation3d[] toArray(List<Rotation3d> list) {
        final Rotation3d[] array = new Rotation3d[list.size()];

        for (int i = 0; i < array.length; i++)
            array[i] = list.get(i);

        return array;
    }

    private Rotation3d extractRotation3d(PhotonTrackedTarget target) {
//        final TargetCorner lowerCorner1 = target.getDetectedCorners().get(0);
//        final TargetCorner lowerCorner2 = target.getDetectedCorners().get(1);
//        if (target.getDetectedCorners().get(3).y < )
//        final Rotation3d correctedPixelRot = getCorrectedPixelRot(target.getDetectedCorners().get())

        return new Rotation3d(
                0,
                Units.degreesToRadians(target.getPitch()),
                Units.degreesToRadians(-target.getYaw())
        );
    }

    private Rotation3d getCorrectedPixelRot(Point point) {
        final Matrix<N3, N3> cameraIntrinsics = photonCamera.getCameraMatrix().orElse(null);
        if (cameraIntrinsics == null)
            return null;

        double fx = cameraIntrinsics.get(0, 0);
        double cx = cameraIntrinsics.get(0, 2);
        double xOffset = cx - point.x;
        double fy = cameraIntrinsics.get(1, 1);
        double cy = cameraIntrinsics.get(1, 2);
        double yOffset = cy - point.y;
        Rotation2d yaw = new Rotation2d(fx, xOffset);
        Rotation2d pitch = new Rotation2d(fy / Math.cos(Math.atan(xOffset / fx)), -yOffset);
        return new Rotation3d(0.0, pitch.getRadians(), yaw.getRadians());
    }
}
