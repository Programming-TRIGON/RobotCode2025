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
    );
    public static final double CORAL_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 0.5;

    private static final Transform3d
            ROBOT_CENTER_TO_OBJECT_DETECTION_CAMERA = new Transform3d(
            new Translation3d(0, 0, 0.77),
            new Rotation3d(0, Units.degreesToRadians(30), 0)
    ),
            ROBOT_CENTER_TO_LEFT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.2233383804584499, 0.2896992413455202, 0.369),
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(10), Units.degreesToRadians(200))
            ),
            ROBOT_CENTER_TO_RIGHT_REEF_TAG_CAMERA = new Transform3d(
                    new Translation3d(-0.19412155946556942, -0.26473108567364695, 0.369),
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
}