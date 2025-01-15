package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;
import frc.trigon.robot.misc.objectdetectioncamera.io.PhotonObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.io.SimulationObjectDetectionCameraIO;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

/**
 * An object detection camera is a class that represents a camera that detects objects other than apriltags, most likely game pieces.
 */
public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private Rotation2d trackedObjectYaw = new Rotation2d();
    private boolean trackedTargetWasVisible = false;
    private int currentTrackedObjectId;

    public ObjectDetectionCamera(String hostname, Rotation2d cameraMountYaw) {
        this.hostname = hostname;
        objectDetectionCameraIO = generateIO(hostname, cameraMountYaw);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
    }

    public int getCurrentTrackedObjectId() {
        return currentTrackedObjectId;
    }

    /**
     * Starts tracking the best visible target of the target ID and remains tracking that target until it is no longer visible.
     * Tracking an object is locking on to one target and allows for you to remain locked on to one target even when there are more objects visible.
     * This should be called periodically.
     * This is used when there is more than one visible object of the target ID and the best target might change as the robot moves.
     * When no objects are visible, the tracking resets to the best target the next time an object of the target ID is visible.
     *
     * @param targetGamePiece the type of game piece to track
     */
    public void trackObject(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        this.currentTrackedObjectId = targetGamePiece.id;
        final boolean hasTargets = hasTargets();
        if (hasTargets && !trackedTargetWasVisible) {
            trackedTargetWasVisible = true;
            trackedObjectYaw = getBestObjectYaw();
            return;
        }

        if (!hasTargets) {
            trackedTargetWasVisible = false;
            return;
        }

        trackedObjectYaw = calculateTrackedObjectYaw();
    }

    public boolean hasTargets() {
        return getTargetObjectYaws().length > 0;
    }

    /**
     * @return the yaw (x-axis position) of the target object
     */
    public Rotation2d getBestObjectYaw() {
        return getTargetObjectYaws()[0];
    }

    /**
     * @return the yaw (x-axis position) of the current tracked object
     */
    public Rotation2d getTrackedObjectYaw() {
        return trackedObjectYaw;
    }

    /**
     * Calculates the yaw (x-axis position) of the object that the camera is currently tracking by finding the target with the least yaw deviation and assuming that it is the same target.
     *
     * @return the yaw of the tracked object
     */
    private Rotation2d calculateTrackedObjectYaw() {
        double closestTargetToTrackedTargetYawDifference = Double.POSITIVE_INFINITY;
        Rotation2d closestToTrackedTargetYaw = new Rotation2d();

        for (Rotation2d currentObjectYaw : getTargetObjectYaws()) {
            final double currentObjectToTrackedTargetYawDifference = Math.abs(currentObjectYaw.getRadians() - trackedObjectYaw.getRadians());
            if (currentObjectToTrackedTargetYawDifference < closestTargetToTrackedTargetYawDifference) {
                closestTargetToTrackedTargetYawDifference = currentObjectToTrackedTargetYawDifference;
                closestToTrackedTargetYaw = currentObjectYaw;
            }
        }

        return closestToTrackedTargetYaw;
    }

    private Rotation2d[] getTargetObjectYaws() {
        if (currentTrackedObjectId >= objectDetectionCameraInputs.visibleObjectYaws.length)
            return new Rotation2d[0];
        return objectDetectionCameraInputs.visibleObjectYaws[currentTrackedObjectId];
    }

    private ObjectDetectionCameraIO generateIO(String hostname, Rotation2d cameraMountYaw) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname, cameraMountYaw);
        return new PhotonObjectDetectionCameraIO(hostname, cameraMountYaw);
    }
}