package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * A simulation object detection camera simulates an object detection camera as well as game pieces on the field and allows for interaction with the game pieces.
 */
public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    public static boolean
            IS_HOLDING_CORAL = true,
            IS_HOLDING_ALGAE = true;
    private static boolean SHOULD_TRACK_CORAL = true;
    private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_VISIBLE_DISTANCE_METERS = 5,
            MINIMUM_VISIBLE_DISTANCE_METERS = 0.05;
    private static final double PICKING_UP_TOLERANCE_METERS = 0.3;
    private static final double CORAL_CENTER_DISTANCE_FROM_GROUND = 0.15;

    private final ArrayList<Translation2d>
            coralOnField = FieldConstants.CORAL_ON_FIELD,
            algaeOnField = FieldConstants.ALGAE_ON_FIELD;
    private Pose3d[]
            heldCoral = new Pose3d[0],
            heldAlgae = new Pose3d[0];
    private boolean
            isDelayingCoralEjection = false,
            isDelayingAlgaeEjection = false;

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Rotation2d closestTargetObjectYaw = calculateClosestVisibleTargetObjectYaw(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
        if (closestTargetObjectYaw == null)
            inputs.hasTargets = false;
        else {
            inputs.hasTargets = true;
            inputs.visibleTargetObjectsYaw = new Rotation2d[]{closestTargetObjectYaw};
        }

        updateHeldGamePiece();

        IS_HOLDING_CORAL = heldCoral != null;
        IS_HOLDING_ALGAE = heldAlgae != null;
        logGamePieces();
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

        final List<Translation2d> visibleTargetObjectsPlacements = calculateVisibleTargetObjectPlacements(robotPose);

        for (Translation2d targetObjectPlacement : visibleTargetObjectsPlacements) {
            final double robotDistanceToObject = calculateObjectDistance(targetObjectPlacement, robotPose);
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
    private List<Translation2d> calculateVisibleTargetObjectPlacements(Pose2d robotPose) {
        final List<Translation2d> targetObjectsOnField = SHOULD_TRACK_CORAL ? coralOnField : algaeOnField;
        final List<Translation2d> visibleTargetObjects = new ArrayList<>();
        int currentIndex = 0;

        for (Translation2d currentTargetObject : targetObjectsOnField) {
            final Rotation2d cameraYawToObject = calculateCameraYawToObject(currentTargetObject, robotPose);
            if (!isWithinHorizontalFOV(cameraYawToObject, robotPose) || !isWithinDistance(targetObjectsOnField.get(0), robotPose))
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
    private Rotation2d calculateCameraYawToObject(Translation2d objectPlacement, Pose2d robotPose) {
        final Translation2d robotToObject = objectPlacement.minus(robotPose.getTranslation());
        return robotToObject.getAngle();
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
    private boolean isWithinDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final double robotToObjectDistanceMeters = calculateObjectDistance(objectPlacement, robotPose);
        return robotToObjectDistanceMeters <= MAXIMUM_VISIBLE_DISTANCE_METERS && robotToObjectDistanceMeters >= MINIMUM_VISIBLE_DISTANCE_METERS;
    }

    /**
     * Calculates the distance between the robot and an object.
     *
     * @param objectPlacement the placement of the object on the field
     * @param robotPose       the position of the robot on the field
     * @return the distance between the robot and the object
     */
    private double calculateObjectDistance(Translation2d objectPlacement, Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(objectPlacement);
    }

    /**
     * Updates the state of the held game pieces (whether it is collecting, ejecting, etc.)
     */
    private void updateHeldGamePiece() {
        updateCoralCollection();
        updateCoralEjection();
        updateHeldCoralPose();

        updateAlgaeCollection();
        updateAlgaeEjection();
        updateHeldAlgaePose();
    }

    /**
     * Handles when a coral should collect by checking the state of the necessary subsystems of the robot and the position of the robot relative to the coral.
     * Also regenerates picked up coral.
     */
    private void updateCoralCollection() {
        if (heldCoral.length > 0 || !isCollectingCoral())
            return;
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d robotTranslation = robotPose.getTranslation();
        for (Translation2d coralPlacement : coralOnField) {
            if (coralPlacement.getDistance(robotTranslation) <= PICKING_UP_TOLERANCE_METERS) {
                heldCoral = new Pose3d[]{calculateHeldCoralPose(robotPose)};
                coralOnField.remove(coralPlacement);
                GeneralCommands.getDelayedCommand(7, () -> coralOnField.add(coralPlacement)).schedule();
                break;
            }
        }
    }

    /**
     * Handles when an algae should collect by checking the state of the necessary subsystems of the robot and the position of the robot relative to the algae.
     * Also regenerates picked up algae.
     */
    private void updateAlgaeCollection() {
        if (heldAlgae.length > 0 || !isCollectingAlgae())
            return;
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Translation2d robotTranslation = robotPose.getTranslation();
        for (Translation2d algaePlacement : algaeOnField) {
            if (algaePlacement.getDistance(robotTranslation) <= PICKING_UP_TOLERANCE_METERS) {
                heldAlgae = new Pose3d[]{calculateHeldAlgaePose(robotPose)};
                algaeOnField.remove(algaePlacement);
                GeneralCommands.getDelayedCommand(7, () -> algaeOnField.add(algaePlacement)).schedule();
                break;
            }
        }
    }

    /**
     * Handles when a held coral should eject from the robot by checking the states of the necessary subsystems.
     */
    private void updateCoralEjection() {
        if (heldCoral.length > 0 || !isEjectingCoral() || isDelayingCoralEjection)
            return;
        isDelayingCoralEjection = true;
        GeneralCommands.getDelayedCommand(0.04, () -> {
            heldCoral = null;
            isDelayingCoralEjection = false;
        }).schedule();
    }

    /**
     * Handles when a game piece should eject from the robot by checking the states of the necessary subsystems.
     */
    private void updateAlgaeEjection() {
        if (heldAlgae.length > 0 || !isEjectingAlgae() || isDelayingAlgaeEjection)
            return;
        isDelayingAlgaeEjection = true;
        GeneralCommands.getDelayedCommand(0.04, () -> {
            heldAlgae = null;
            isDelayingAlgaeEjection = false;
        }).schedule();
    }

    /**
     * Updates the position of the held coral so that it stays inside the robot.
     */
    private void updateHeldCoralPose() {
        if (heldCoral.length == 0)
            return;
        heldCoral = new Pose3d[]{calculateHeldCoralPose(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose())};
    }

    /**
     * Updates the position of the held algae so that it stays inside the robot.
     */
    private void updateHeldAlgaePose() {
        if (heldAlgae.length == 0)
            return;
        heldAlgae = new Pose3d[]{calculateHeldAlgaePose(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose())};
    }

    private boolean isCollectingCoral() {
        return RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.COLLECT) ||
                RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.COLLECT_FEEDER);
    }

    private boolean isCollectingAlgae() {
        return true;
    }

    private boolean isEjectingCoral() {
        return false;//TODO: Implement for when a coral should exit the robot
    }

    private boolean isEjectingAlgae() {
        return false;//TODO: Implement for when an algae should exit the robot
    }

    /**
     * Calculate the position of the held coral relative to the field.
     *
     * @param robotPose the position of the robot on the field
     * @return the position of the coral relative to the field
     */
    private Pose3d calculateHeldCoralPose(Pose2d robotPose) {
        final Pose3d robotPose3d = new Pose3d(robotPose);
        final Pose3d robotRelativeHeldGamePiecePosition = new Pose3d();// TODO:Set position
        return robotPose3d.plus(toTransform(robotRelativeHeldGamePiecePosition));
    }

    /**
     * Calculate the position of the held algae relative to the field.
     *
     * @param robotPose the position of the robot on the field
     * @return the position of the algae relative to the field
     */
    private Pose3d calculateHeldAlgaePose(Pose2d robotPose) {
        final Pose3d robotPose3d = new Pose3d(robotPose);
        final Pose3d robotRelativeHeldGamePiecePosition = new Pose3d();// TODO:Set position
        return robotPose3d.plus(toTransform(robotRelativeHeldGamePiecePosition));
    }

    /**
     * Changes a Pose3d into a Transform3d.
     *
     * @param pose the target Pose3d
     * @return the Transform3d
     */
    private Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Logs the position of all the game pieces on the field and in the robot.
     */
    private void logGamePieces() {
        Logger.recordOutput("Poses/GamePieces/HeldCoral", heldCoral);
        Logger.recordOutput("Poses/GamePieces/HeldAlgae", heldAlgae);

        Logger.recordOutput("Poses/GamePieces/CoralOnField", toPosesArray(coralOnField));
        Logger.recordOutput("Poses/GamePieces/AlgaeOnField", toPosesArray(algaeOnField));
    }

    /**
     * Changes a list of Translation2ds to an array of Pose3ds with the height of {@link #CORAL_CENTER_DISTANCE_FROM_GROUND}.
     *
     * @param translationsList the list of Translation2ds
     * @return the array of Pose3ds
     */
    private Pose3d[] toPosesArray(List<Translation2d> translationsList) {
        final Pose3d[] posesArray = new Pose3d[translationsList.size()];
        for (int i = 0; i < translationsList.size(); i++) {
            final Translation2d translation = translationsList.get(i);
            posesArray[i] = new Pose3d(translation.getX(), translation.getY(), CORAL_CENTER_DISTANCE_FROM_GROUND, new Rotation3d());
        }
        return posesArray;
    }
}