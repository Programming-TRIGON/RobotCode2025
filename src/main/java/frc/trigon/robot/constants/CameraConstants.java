package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.poseestimator.StandardDeviations;

public class CameraConstants {
    private static final StandardDeviations
            REEF_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
            0.015,
            0.01
    ),
            FEEDER_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
                    0.005,
                    0.01
            );

    private static final Transform3d
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA = new Transform3d(
            new Translation3d(0.046, 0, 0.9),
            new Rotation3d(0, Units.degreesToRadians(18), 0)
    ),
            ROBOT_CENTER_TO_LEFT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.224, 0.276, 0.369),
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(10), Units.degreesToRadians(200))
            ),
            ROBOT_CENTER_TO_RIGHT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.224, -0.276, 0.369),
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(10), Units.degreesToRadians(160))
            ),
            ROBOT_CENTER_TO_FEEDER_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.005, 0, 0.828),
                    new Rotation3d(0, Units.degreesToRadians(-35), 0)
            );

    public static final ObjectDetectionCamera OBJECT_DETECTION_CAMERA = new ObjectDetectionCamera(
            "ObjectDetectionCamera",
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA
    );
    public static final AprilTagCamera
            LEFT_REEF_TAG_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "LeftReefTagCamera",
            ROBOT_CENTER_TO_LEFT_REEF_TAG_CAMERA,
            REEF_TAG_CAMERA_STANDARD_DEVIATIONS
    ),
            RIGHT_REEF_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "RightReefTagCamera",
                    ROBOT_CENTER_TO_RIGHT_REEF_TAG_CAMERA,
                    REEF_TAG_CAMERA_STANDARD_DEVIATIONS
            );
//            FEEDER_TAG_CAMERA = new AprilTagCamera(
//                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
//                    "CoralStationTagCamera",
//                    ROBOT_CENTER_TO_FEEDER_TAG_CAMERA,
//                    FEEDER_TAG_CAMERA_STANDARD_DEVIATIONS
//            );
}