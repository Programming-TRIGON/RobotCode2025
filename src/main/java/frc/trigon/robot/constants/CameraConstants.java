package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;

public class CameraConstants {
    public static final ObjectDetectionCamera CORAL_DETECTION_CAMERA = new ObjectDetectionCamera(
            "CoralDetectionCamera",
            new Transform3d(
                    new Translation3d(0, 0, 0.8),
                    new Rotation3d(0, Units.degreesToRadians(30), 0)
            )
    );
    //TODO: implement the rest of CameraConstants
}