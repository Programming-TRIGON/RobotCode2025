package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;

import java.util.ArrayList;

/**
 * A simulation object detection camera simulates an object detection camera as well as game pieces on the field and allows for interaction with the game pieces.
 */
public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_VISIBLE_DISTANCE_METERS = 5,
            MINIMUM_VISIBLE_DISTANCE_METERS = 0.05;

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final ArrayList<SimulatedGamePiece>
                visibleCoral = calculateVisibleGamePieces(robotPose, 0),
                visibleAlgae = calculateVisibleGamePieces(robotPose, 1);

        inputs.hasCoralTarget = !visibleCoral.isEmpty();
        inputs.hasAlgaeTarget = !visibleAlgae.isEmpty();

        if (!inputs.hasCoralTarget && !inputs.hasAlgaeTarget) {
            updateNoNewResultInputs(inputs);
            return;
        }

        updateHasNewResultInputs(visibleCoral, visibleAlgae, robotPose, inputs);
    }

    /**
     * Calculates the placements of all visible objects by checking if they are within range and within the horizontal FOV.
     *
     * @param robotPose the position of the robot on the field
     * @return the placements of the visible objects
     */
    private ArrayList<SimulatedGamePiece> calculateVisibleGamePieces(Pose2d robotPose, int targetId) {
        final ArrayList<SimulatedGamePiece> gamePiecesOnField = targetId == 0 ? SimulationFieldHandler.getSimulatedCoral() : SimulationFieldHandler.getSimulatedAlgae();
        final ArrayList<SimulatedGamePiece> visibleTargetObjects = new ArrayList<>();
        int currentIndex = 0;

        for (SimulatedGamePiece currentObject : gamePiecesOnField) {
            final Rotation2d cameraYawToObject = calculateCameraYawToObject(currentObject, robotPose);
            if (!isWithinHorizontalFOV(cameraYawToObject, robotPose) || !isWithinDistance(currentObject, robotPose))
                continue;
            visibleTargetObjects.add(currentIndex, currentObject);
            currentIndex++;
        }
        return visibleTargetObjects;
    }

    private void updateNoNewResultInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        inputs.visibleCoralYaws = new Rotation2d[0];
        inputs.visibleAlgaeYaws = new Rotation2d[0];
    }

    private void updateHasNewResultInputs(ArrayList<SimulatedGamePiece> visibleCoral, ArrayList<SimulatedGamePiece> visibleAlgae, Pose2d robotPose, ObjectDetectionCameraInputsAutoLogged inputs) {
        if (inputs.hasCoralTarget) {
            final SimulatedGamePiece closestCoral = calculateClosestVisibleObject(robotPose, visibleCoral);
            inputs.visibleCoralYaws[0] = calculateCameraYawToObject(closestCoral, robotPose);
        }
        if (inputs.hasAlgaeTarget) {
            final SimulatedGamePiece closestAlgae = calculateClosestVisibleObject(robotPose, visibleAlgae);
            inputs.visibleAlgaeYaws[0] = calculateCameraYawToObject(closestAlgae, robotPose);
        }
    }

    /**
     * Calculates the yaw of the closest visible target game piece relative to the camera by calculating the yaw of all target objects visible to the camera and returning the one with the smallest yaw deviation from the camera's center.
     *
     * @param robotPose the pose of the robot on the field
     * @return the yaw of the closest visible target object
     */
    private SimulatedGamePiece calculateClosestVisibleObject(Pose2d robotPose, ArrayList<SimulatedGamePiece> visibleGamePieces) {
        double closestObjectDistance = Double.POSITIVE_INFINITY;
        SimulatedGamePiece closestObject = null;

        for (SimulatedGamePiece targetObjectPlacement : visibleGamePieces) {
            final double robotDistanceToObject = targetObjectPlacement.getDistanceMeters(new Pose3d(robotPose));
            if (robotDistanceToObject < closestObjectDistance) {
                closestObjectDistance = robotDistanceToObject;
                closestObject = targetObjectPlacement;
            }
        }

        return closestObject;
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