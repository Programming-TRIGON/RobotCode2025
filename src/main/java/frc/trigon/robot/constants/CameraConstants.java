package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.poseestimator.StandardDeviations;

public class CameraConstants {
    //TODO: implement CameraConstants
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d(),
            new Rotation3d()
    );
    public static final AprilTagCamera CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "RearTagCamera",
            ROBOT_TO_CAMERA,
            new StandardDeviations(0.005, 0.01)
    );
}
