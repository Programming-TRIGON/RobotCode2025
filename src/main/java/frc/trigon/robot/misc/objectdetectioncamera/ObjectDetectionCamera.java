package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetectioncamera.io.PhotonObjectDetectionCameraIO;
import frc.trigon.robot.misc.objectdetectioncamera.io.SimulationObjectDetectionCameraIO;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

/**
 * An object detection camera is a class that represents a camera that detects objects other than apriltags, most likely game pieces.
 */
public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private final Transform3d robotToCamera;
    private Rotation3d trackedObjectRotation = new Rotation3d();
    private boolean trackedTargetWasVisible = false;

    public ObjectDetectionCamera(String hostname, Transform3d robotToCamera) {
        this.hostname = hostname;
        this.robotToCamera = robotToCamera;
        this.objectDetectionCameraIO = generateIO(hostname, robotToCamera);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
    }

    /**
     * Calculates the position of the tracked object on the field from the rotation of the object.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the tracked object's 3d position on the field
     */
    public Translation3d calculateTrackedObjectPositionOnField() {
        return calculateObjectPositionFromRotation(trackedObjectRotation);
    }


    /**
     * Calculates the position of the best object on the field from the rotation of the object.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the best object's 3d position on the field
     */
    public Translation3d calculateBestObjectPositionOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return calculateObjectPositionFromRotation(calculateBestObjectRotation(targetGamePiece));
    }

    /**
     * Calculates the position of the object on the field from the rotation of the object.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @param objectRotation the object's 3d rotation relative to the camera
     * @return the object's 3d position on the field
     */
    public Translation3d calculateObjectPositionFromRotation(Rotation3d objectRotation) {
        final Pose3d cameraPose = new Pose3d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose()).plus(robotToCamera);
        final Pose3d objectRotationStart = cameraPose.plus(new Transform3d(0, 0, 0, objectRotation));

        final double cameraZ = cameraPose.getTranslation().getZ();
        final double objectPitchSin = Math.sin(objectRotationStart.getRotation().getY());
        final double transformX = cameraZ / objectPitchSin;
        final Transform3d objectRotationStartToGround = new Transform3d(transformX, 0, 0, new Rotation3d());

        return objectRotationStart.transformBy(objectRotationStartToGround).getTranslation();
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
        final boolean hasTargets = hasTargets(targetGamePiece);
        if (hasTargets && !trackedTargetWasVisible) {
            trackedTargetWasVisible = true;
            trackedObjectRotation = calculateBestObjectRotation(targetGamePiece);
            return;
        }

        if (!hasTargets) {
            trackedTargetWasVisible = false;
            return;
        }

        trackedObjectRotation = calculateTrackedObjectYaw(targetGamePiece);
    }

    public boolean hasTargets(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return objectDetectionCameraInputs.hasTarget[targetGamePiece.id];
    }

    /**
     * @return the yaw (x-axis position) of the target object
     */
    public Rotation3d calculateBestObjectRotation(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Rotation3d[] targetObjectsRotations = getTargetObjectsRotations(targetGamePiece.id);
        Rotation3d bestObjectRotation = targetObjectsRotations[0];

        for (int i = 1; i < targetObjectsRotations.length; i++) {
            final Rotation3d currentObjectRotation = targetObjectsRotations[i];
            final double bestObjectDistance = calculateDistance(bestObjectRotation, ObjectDetectionCameraConstants.BEST_ROTATION);
            final double currentObjectDistance = calculateDistance(currentObjectRotation, ObjectDetectionCameraConstants.BEST_ROTATION);

            if (currentObjectDistance < bestObjectDistance)
                bestObjectRotation = currentObjectRotation;
        }

        return bestObjectRotation;
    }

    /**
     * @return the 3d rotation of the object that the camera is currently tracking, relative to the camera
     */
    public Rotation3d getTrackedObjectRotation() {
        return trackedObjectRotation;
    }

    private double calculateDistance(Rotation3d from, Rotation3d to) {
        return from.minus(to).getAngle();
    }

    /**
     * Calculates the 3d rotation of the object that the camera is currently tracking by finding the target with the least rotation deviation and assuming that it is the same target.
     *
     * @return the 3d rotation of the tracked object
     */
    private Rotation3d calculateTrackedObjectYaw(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        final Rotation3d[] targetObjectsRotations = getTargetObjectsRotations(targetGamePiece.id);
        Rotation3d closestObjectRotation = targetObjectsRotations[0];

        for (int i = 1; i < targetObjectsRotations.length; i++) {
            final Rotation3d currentObjectRotation = targetObjectsRotations[i];
            final double closestObjectDistance = calculateDistance(closestObjectRotation, trackedObjectRotation);
            final double currentObjectDistance = calculateDistance(currentObjectRotation, trackedObjectRotation);

            if (currentObjectDistance < closestObjectDistance)
                closestObjectRotation = currentObjectRotation;
        }

        return closestObjectRotation;
    }

    private Rotation3d[] getTargetObjectsRotations(int objectID) {
        return objectDetectionCameraInputs.visibleObjectRotations[objectID];
    }

    private ObjectDetectionCameraIO generateIO(String hostname, Transform3d robotToCamera) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname, robotToCamera);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}