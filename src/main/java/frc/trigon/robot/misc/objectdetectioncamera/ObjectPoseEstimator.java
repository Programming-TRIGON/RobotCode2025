package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

import java.util.ArrayList;
import java.util.HashMap;

public class ObjectPoseEstimator {
    private final HashMap<Translation2d, Double> objectPositions;
    private final ObjectDetectionCamera camera;
    private final double deleteThresholdSeconds;
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;

    /**
     * Constructs an ObjectPoseEstimator for estimating the positions of objects detected by a camera.
     *
     * @param deleteThresholdSeconds the time in seconds after which an object is considered old and removed
     * @param gamePieceType          the type of game piece to track
     * @param camera                 the camera used for detecting objects
     */
    public ObjectPoseEstimator(double deleteThresholdSeconds, SimulatedGamePieceConstants.GamePieceType gamePieceType, ObjectDetectionCamera camera) {
        this.deleteThresholdSeconds = deleteThresholdSeconds;
        this.gamePieceType = gamePieceType;
        this.camera = camera;
        this.objectPositions = new HashMap<>();
    }

    /**
     * Updates the object positions based on the camera's detected objects.
     * Removes objects that have not been detected for a certain period of time.
     * This method should be called periodically to keep the object positions up to date.
     */
    public void update() {
        updateObjectRotations();
        removeOldObjects();
    }

    /**
     * Gets the position of all known objects on the field.
     *
     * @return a list of Translation2d representing the positions of objects on the field
     */
    public ArrayList<Translation2d> getObjectsOnField() {
        return new ArrayList<>(objectPositions.keySet());
    }

    private void updateObjectRotations() {
        final double currentTimestamp = Timer.getTimestamp();
        for (Translation2d cameraObjectOnField : camera.getObjectPositionsOnField(gamePieceType)) {
            Translation2d closestObjectToObjectPositionOnField = new Translation2d();
            double closestObjectDistanceMeters = Double.POSITIVE_INFINITY;

            for (Translation2d knownObject : objectPositions.keySet()) {
                final double currentObjectDistanceMeters = cameraObjectOnField.getDistance(knownObject);
                if (currentObjectDistanceMeters < closestObjectDistanceMeters) {
                    closestObjectDistanceMeters = currentObjectDistanceMeters;
                    closestObjectToObjectPositionOnField = knownObject;
                }
            }

            if (closestObjectDistanceMeters < ObjectDetectionCameraConstants.TRACKED_OBJECT_TOLERANCE_METERS && objectPositions.get(closestObjectToObjectPositionOnField) != currentTimestamp)
                objectPositions.remove(closestObjectToObjectPositionOnField);
            objectPositions.put(cameraObjectOnField, currentTimestamp);
        }
    }

    private void removeOldObjects() {
        objectPositions.entrySet().removeIf(entry -> isTooOld(entry.getValue()));
    }

    private boolean isTooOld(double timestamp) {
        return Timer.getTimestamp() - timestamp > deleteThresholdSeconds;
    }
}