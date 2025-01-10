package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
    }

    /**
     * Sets which type of game piece to track.
     *
     * @param shouldTrackCoral whether to track coral or algae
     */
    protected void setTrackingObject(boolean shouldTrackCoral) {
    }

    @AutoLog
    public static class ObjectDetectionCameraInputs {
        public boolean hasTargets = false;
        /**
         * An array that contains the yaw of all visible targets. The best target is first.
         */
        public Rotation2d[] visibleTargetObjectsYaw = new Rotation2d[0];
    }
}
