package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
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
    private final Transform3d robotCenterToCamera;
    private final Timer switchToNewTargetTimer = new Timer();
    private Rotation3d trackedObjectRotation = new Rotation3d();
    private boolean trackedTargetWasVisible = false;

    public ObjectDetectionCamera(String hostname, Transform3d robotCenterToCamera) {
        this.hostname = hostname;
        this.robotCenterToCamera = robotCenterToCamera;
        this.objectDetectionCameraIO = generateIO(hostname, robotCenterToCamera);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
        logVisibleObjects();
    }

    public boolean isCurrentTrackedGamePieceVisibleWithinTimeout() {
        return !switchToNewTargetTimer.hasElapsed(ObjectDetectionCameraConstants.SWITCH_TO_NEW_TARGET_TIMEOUT_SECONDS);
    }

    /**
     * Calculates the position of the tracked object on the field from the rotation of the object.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the tracked object's 2d position on the field (z can be assumed to be 0)
     */
    public Translation2d calculateTrackedObjectPositionOnField() {
        if (trackedObjectRotation == null)
            return null;
        Logger.recordOutput("TrackedObjectPosition", new Translation3d(calculateObjectPositionFromRotation(trackedObjectRotation)));
        return calculateObjectPositionFromRotation(trackedObjectRotation);
    }

    /**
     * Calculates the position of the best object on the field from the rotation of the object.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @return the best object's 2d position on the field (z can be assumed to be 0)
     */
    public Translation2d calculateBestObjectPositionOnField(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
        return calculateObjectPositionFromRotation(calculateBestObjectRotation(targetGamePiece));
    }

    /**
     * Calculates the position of the object on the field from the rotation of the object.
     * This assumes the object is on the ground.
     * Once it is known that the object is on the ground,
     * one can simply find the transform from the camera to the ground and apply it to the object's rotation.
     *
     * @param objectRotation the object's 3d rotation relative to the camera
     * @return the object's 2d position on the field (z can be assumed to be 0)
     */
    public Translation2d calculateObjectPositionFromRotation(Rotation3d objectRotation) {
        final Pose3d cameraPose = new Pose3d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose()).plus(robotCenterToCamera);
        objectRotation = new Rotation3d(objectRotation.getX(), -objectRotation.getY(), objectRotation.getZ());
        final Pose3d objectRotationStart = cameraPose.plus(new Transform3d(0, 0, 0, objectRotation));

        final double cameraZ = cameraPose.getTranslation().getZ();
        final double objectPitchSin = Math.sin(objectRotationStart.getRotation().getY());
        final double transformX = cameraZ / objectPitchSin;
        final Transform3d objectRotationStartToGround = new Transform3d(transformX, 0, 0, new Rotation3d());

        return objectRotationStart.transformBy(objectRotationStartToGround).getTranslation().toTranslation2d();
    }

    public void initializeTracking() {
        trackedTargetWasVisible = false;
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
            switchToNewTargetTimer.reset();
            trackedTargetWasVisible = true;
            trackedObjectRotation = calculateBestObjectRotation(targetGamePiece);
            return;
        }

        if (!isCurrentTrackedGamePieceVisibleWithinTimeout()) {
            trackedObjectRotation = null;
            trackedTargetWasVisible = false;
            return;
        }

        if (!hasTargets) {
            switchToNewTargetTimer.start();
            return;
        }

        switchToNewTargetTimer.reset();
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
            final double bestObjectDifference = Math.abs(bestObjectRotation.getY() - ObjectDetectionCameraConstants.BEST_PITCH.getRadians());
            final double currentObjectDifference = Math.abs(currentObjectRotation.getY() - ObjectDetectionCameraConstants.BEST_PITCH.getRadians());

            if (currentObjectDifference < bestObjectDifference)
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

    private void logVisibleObjects() {
        for (int i = 0; i < SimulatedGamePieceConstants.GamePieceType.values().length; i++)
            logVisibleTargetObject(i);
    }

    private void logVisibleTargetObject(int targetID) {
        final Rotation3d[] visibleTargetObjectRotations = getTargetObjectsRotations(targetID);
        final Pose2d[] visibleTargetObjectPoses = new Pose2d[visibleTargetObjectRotations.length];
        for (int i = 0; i < visibleTargetObjectPoses.length; i++) {
            final Rotation3d objectRotation = visibleTargetObjectRotations[i];
            final Translation2d objectPose = calculateObjectPositionFromRotation(objectRotation);
            visibleTargetObjectPoses[i] = new Pose2d(objectPose, objectRotation.toRotation2d());
        }
        Logger.recordOutput("Visible" + SimulatedGamePieceConstants.GamePieceType.getNameFromID(targetID), visibleTargetObjectPoses);
    }

    private ObjectDetectionCameraIO generateIO(String hostname, Transform3d robotCenterToCamera) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname, robotCenterToCamera);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}