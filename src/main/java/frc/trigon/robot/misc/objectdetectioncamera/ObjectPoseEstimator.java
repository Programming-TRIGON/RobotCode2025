package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

import java.util.ArrayList;
import java.util.HashMap;

public class ObjectPoseEstimator extends SubsystemBase {
    private final HashMap<Translation2d, Double> knownObjectPositions;
    private final ObjectDetectionCamera[] cameras;
    private final double deletionThresholdSeconds;
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;

    /**
     * Constructs an ObjectPoseEstimator for estimating the positions of objects detected by cameras.
     *
     * @param deletionThresholdSeconds the time in seconds after which an object is considered old and removed
     * @param gamePieceType            the type of game piece to track
     * @param cameras                  the cameras used for detecting objects
     */
    public ObjectPoseEstimator(double deletionThresholdSeconds, SimulatedGamePieceConstants.GamePieceType gamePieceType, ObjectDetectionCamera... cameras) {
        this.deletionThresholdSeconds = deletionThresholdSeconds;
        this.gamePieceType = gamePieceType;
        this.cameras = cameras;
        this.knownObjectPositions = new HashMap<>();
    }

    /**
     * Updates the object positions based on the cameras's detected objects.
     * Removes objects that have not been detected for {@link ObjectPoseEstimator#deletionThresholdSeconds}.
     */
    @Override
    public void periodic() {
        updateObjectRotations();
        removeOldObjects();
    }

    /**
     * Gets the position of all known objects on the field.
     *
     * @return a list of Translation2d representing the positions of objects on the field
     */
    public ArrayList<Translation2d> getObjectsOnField() {
        return new ArrayList<>(knownObjectPositions.keySet());
    }

    /**
     * Removes the closest object to the robot from the list of objects in the pose estimator.
     */
    public void removeClosestObjectToRobot() {
        final Translation2d closestObject = getClosestObjectToRobot();
        knownObjectPositions.remove(closestObject);
    }

    /**
     * Removes the closest object to the intake from the list of objects in the pose estimator.
     *
     * @param intakeTransform the transform of the intake relative to the robot
     */
    public void removeClosestObjectToIntake(Transform2d intakeTransform) {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        final Translation2d intakePosition = robotPose.transformBy(intakeTransform).getTranslation();
        knownObjectPositions.remove(getClosestObjectToPose(intakePosition));
    }

    /**
     * Removes the closest object to a given pose from the list of objects in the pose estimator.
     *
     * @param pose the pose to which the removed object is closest
     */
    public void removeClosestObjectToPose(Translation2d pose) {
        final Translation2d closestObject = getClosestObjectToPose(pose);
        knownObjectPositions.remove(closestObject);
    }

    /**
     * Removes a specific object from the list of known objects in the pose estimator.
     *
     * @param objectPosition the position of the object to be removed
     */
    public void removeObject(Translation2d objectPosition) {
        knownObjectPositions.remove(objectPosition);
    }

    /**
     * Gets the position of the closest object on the field from the 3D rotation of the object relative to the cameras.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the best object's 2D position on the field (z is assumed to be 0)
     */
    public Translation2d getClosestObjectToRobot() {
        return getClosestObjectToPose(RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    /**
     * Gets the position of the closest object to a given pose on the field.
     *
     * @param pose the pose to which the returned object is closest
     * @return the closest object's position on the field, or null if no objects are known
     */
    public Translation2d getClosestObjectToPose(Translation2d pose) {
        final Translation2d[] objectsTranslations = knownObjectPositions.keySet().toArray(Translation2d[]::new);
        if (knownObjectPositions.isEmpty())
            return null;
        Translation2d bestObjectTranslation = objectsTranslations[0];

        for (int i = 1; i < objectsTranslations.length; i++) {
            final Translation2d currentObjectTranslation = objectsTranslations[i];
            final double bestObjectDifference = pose.getDistance(bestObjectTranslation);
            final double currentObjectDifference = pose.getDistance(currentObjectTranslation);
            if (currentObjectDifference < bestObjectDifference)
                bestObjectTranslation = currentObjectTranslation;
        }
        return bestObjectTranslation;
    }

    private void updateObjectRotations() {
        final double currentTimestamp = Timer.getTimestamp();
        for (ObjectDetectionCamera camera : this.cameras) {
            for (Translation2d visibleObject : camera.getObjectPositionsOnField(gamePieceType)) {
                Translation2d closestObjectToVisibleObject = new Translation2d();
                double closestObjectToVisibleObjectDistanceMeters = Double.POSITIVE_INFINITY;

                for (Translation2d knownObject : knownObjectPositions.keySet()) {
                    final double currentObjectDistanceMeters = visibleObject.getDistance(knownObject);
                    if (currentObjectDistanceMeters < closestObjectToVisibleObjectDistanceMeters) {
                        closestObjectToVisibleObjectDistanceMeters = currentObjectDistanceMeters;
                        closestObjectToVisibleObject = knownObject;
                    }
                }

                if (closestObjectToVisibleObjectDistanceMeters < ObjectDetectionCameraConstants.TRACKED_OBJECT_TOLERANCE_METERS && knownObjectPositions.get(closestObjectToVisibleObject) != currentTimestamp)
                    knownObjectPositions.remove(closestObjectToVisibleObject);
                knownObjectPositions.put(visibleObject, currentTimestamp);
            }
        }
    }

    private void removeOldObjects() {
        knownObjectPositions.entrySet().removeIf(entry -> isTooOld(entry.getValue()));
    }

    private boolean isTooOld(double timestamp) {
        return Timer.getTimestamp() - timestamp > deletionThresholdSeconds;
    }
}