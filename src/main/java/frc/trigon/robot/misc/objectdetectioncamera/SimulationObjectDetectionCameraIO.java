package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.objectdetectioncamera.simulatedfield.SimulatedGamePiece;
import frc.trigon.robot.misc.objectdetectioncamera.simulatedfield.SimulationFieldHandler;

import java.util.ArrayList;

/**
 * A simulation object detection camera simulates an object detection camera as well as game pieces on the field and allows for interaction with the game pieces.
 */
public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private static boolean SHOULD_TRACK_CORAL = true;
    private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_VISIBLE_DISTANCE_METERS = 5,
            MINIMUM_VISIBLE_DISTANCE_METERS = 0.05;

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Rotation2d closestTargetObjectYaw = calculateClosestVisibleTargetObjectYaw(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
        if (closestTargetObjectYaw == null)
            inputs.hasTargets = false;
        else {
            inputs.hasTargets = true;
            inputs.visibleTargetObjectsYaw = new Rotation2d[]{closestTargetObjectYaw};
        }
    }

    @Override
    protected void setTrackingObject(boolean shouldTrackCoral) {
        SHOULD_TRACK_CORAL = shouldTrackCoral;
    }

    /**
     * Calculates the yaw of the closest visible target game piece relative to the camera by calculating the yaw of all target objects visible to the camera and returning the one with the smallest yaw deviation from the camera's center.
     *
     * @param robotPose the pose of the robot on the field
     * @return the yaw of the closest visible target object
     */
    private Rotation2d calculateClosestVisibleTargetObjectYaw(Pose2d robotPose) {
        Rotation2d closestObjectYaw = null;
        double closestObjectDistance = Double.POSITIVE_INFINITY;

        final ArrayList<SimulatedGamePiece> visibleTargetObjectsPlacements = calculateVisibleTargetObjectPlacements(robotPose);

        for (SimulatedGamePiece targetObjectPlacement : visibleTargetObjectsPlacements) {
            final double robotDistanceToObject = targetObjectPlacement.getDistanceMeters(new Pose3d(robotPose));
            if (robotDistanceToObject < closestObjectDistance) {
                closestObjectYaw = calculateCameraYawToObject(targetObjectPlacement, robotPose).minus(robotPose.getRotation());
                closestObjectDistance = robotDistanceToObject;
            }
        }

        return closestObjectYaw;
    }

    /**
     * Calculates the placements of all visible objects by checking if they are within range and within the horizontal FOV.
     *
     * @param robotPose the position of the robot on the field
     * @return the placements of the visible objects
     */
    private ArrayList<SimulatedGamePiece> calculateVisibleTargetObjectPlacements(Pose2d robotPose) {
        final ArrayList<SimulatedGamePiece> targetObjectsOnField = SHOULD_TRACK_CORAL ? SimulationFieldHandler.getSimulatedCoral() : SimulationFieldHandler.getSimulatedAlgae();
        final ArrayList<SimulatedGamePiece> visibleTargetObjects = new ArrayList<>();
        int currentIndex = 0;

        for (SimulatedGamePiece currentTargetObject : targetObjectsOnField) {
            final Rotation2d cameraYawToObject = calculateCameraYawToObject(currentTargetObject, robotPose);
            if (!isWithinHorizontalFOV(cameraYawToObject, robotPose) || !isWithinDistance(currentTargetObject, robotPose))
                continue;
            visibleTargetObjects.add(currentIndex, currentTargetObject);
            currentIndex++;
        }
        return visibleTargetObjects;
    }

    /**
     * Calculates the difference in yaw from an object to the camera's yaw.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return the yaw of the object relative to the camera
     */
    private Rotation2d calculateCameraYawToObject(SimulatedGamePiece objectPlacement, Pose2d robotPose) {
        final Transform2d robotToObject = objectPlacement.getPose().toPose2d().minus(robotPose);
        return robotToObject.getRotation();
    }

    /**
     * Checks if an object is within the field-of-view of the camera.
     *
     * @param objectYaw the yaw of the object relative to the camera
     * @param robotPose the position of the robot on the field
     * @return if the object is within the field-of-view of the camera
     */
    private boolean isWithinHorizontalFOV(Rotation2d objectYaw, Pose2d robotPose) {
        return Math.abs(objectYaw.minus(robotPose.getRotation()).getRadians()) <= CAMERA_HORIZONTAL_FOV.getRadians() / 2;
    }

    /**
     * Checks if an object is within the distance range of the camera.
     * The distance range is set at the top of this class as {@link #MINIMUM_VISIBLE_DISTANCE_METERS} and {@link #MINIMUM_VISIBLE_DISTANCE_METERS}.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return if the object is withing the distance range of the camera
     */
    private boolean isWithinDistance(SimulatedGamePiece objectPlacement, Pose2d robotPose) {
        final double robotToObjectDistanceMeters = objectPlacement.getDistanceMeters(new Pose3d(robotPose));
        return robotToObjectDistanceMeters <= MAXIMUM_VISIBLE_DISTANCE_METERS && robotToObjectDistanceMeters >= MINIMUM_VISIBLE_DISTANCE_METERS;
    }
}