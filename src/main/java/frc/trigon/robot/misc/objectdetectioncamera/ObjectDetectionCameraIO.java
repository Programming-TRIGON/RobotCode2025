package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class ObjectDetectionCameraInputs {
        public boolean hasCoralTarget = false;
        public boolean hasAlgaeTarget = false;
        public Rotation2d[][] visibleObjectYaws = new Rotation2d[0][0];
    }
}
