package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * Handles the simulation of game pieces.
 */
public class SimulationFieldHandler {
    private static final ArrayList<SimulatedGamePiece>
            CORAL_ON_FIELD = SimulatedGamePieceConstants.CORAL_ON_FIELD,
            ALGAE_ON_FIELD = SimulatedGamePieceConstants.ALGAE_ON_FIELD;
    private static boolean
            CAN_EJECT_CORAL = false,
            CAN_EJECT_ALGAE = false;
    private static Integer
            HELD_CORAL_INDEX = null,
            HELD_ALGAE_INDEX = null;

    public static ArrayList<SimulatedGamePiece> getSimulatedCoral() {
        return CORAL_ON_FIELD;
    }

    public static ArrayList<SimulatedGamePiece> getSimulatedAlgae() {
        return ALGAE_ON_FIELD;
    }

    public static boolean isHoldingCoral() {
        return HELD_CORAL_INDEX != null;
    }

    public static boolean isHoldingAlgae() {
        return HELD_ALGAE_INDEX != null;
    }

    public static void setCanEjectCoral(boolean canEject) {
        CAN_EJECT_CORAL = canEject;
    }

    public static void setCanEjectAlgae(boolean canEject) {
        CAN_EJECT_ALGAE = canEject;
    }

    public static void update() {
        updateGamePieces();
        SimulationScoringHandler.update();
        logGamePieces();
    }

    /**
     * Updates the state of all game pieces.
     */
    private static void updateGamePieces() {
        updateGamePiecesPeriodically();
        updateCollection();
        updateEjection();
        updateHeldGamePiecePoses();
    }

    /**
     * Logs the position of all the game pieces.
     */
    private static void logGamePieces() {
        Logger.recordOutput("Poses/GamePieces/Corals", mapSimulatedGamePieceListToPoseArray(CORAL_ON_FIELD));
        Logger.recordOutput("Poses/GamePieces/Algae", mapSimulatedGamePieceListToPoseArray(ALGAE_ON_FIELD));
    }

    private static void updateGamePiecesPeriodically() {
        for (SimulatedGamePiece coral : CORAL_ON_FIELD)
            coral.updatePeriodically();
        for (SimulatedGamePiece algae : ALGAE_ON_FIELD)
            algae.updatePeriodically();
    }

    private static void updateCollection() {
        final Pose3d robotPose = new Pose3d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
        final Pose3d
                coralCollectionPose = robotPose.plus(toTransform(RobotContainer.CORAL_INTAKE.calculateCoralCollectionPose())),
                algaeCollectionPose = robotPose.plus(toTransform(new Pose3d()));

        if (isCollectingCoral() && HELD_CORAL_INDEX == null)
            HELD_CORAL_INDEX = getIndexOfCollectedGamePiece(coralCollectionPose, CORAL_ON_FIELD, SimulatedGamePieceConstants.CORAL_INTAKE_TOLERANCE_METERS);
        if (isCollectingAlgae() && HELD_ALGAE_INDEX == null)
            HELD_ALGAE_INDEX = getIndexOfCollectedGamePiece(algaeCollectionPose, ALGAE_ON_FIELD, SimulatedGamePieceConstants.ALGAE_INTAKE_TOLERANCE_METERS);
    }

    /**
     * Gets the index of the game piece that is being collected.
     *
     * @param collectionPose the pose of the collection mechanism
     * @param gamePieceList  the list of game pieces
     * @return the index of the game piece that is being collected
     */
    private static Integer getIndexOfCollectedGamePiece(Pose3d collectionPose, ArrayList<SimulatedGamePiece> gamePieceList, double intakeTolerance) {
        for (SimulatedGamePiece gamePiece : gamePieceList)
            if (gamePiece.getDistanceMeters(collectionPose) <= intakeTolerance)
                return gamePieceList.indexOf(gamePiece);
        return null;
    }

    private static boolean isCollectingCoral() {
        return RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.COLLECT);
    }

    private static boolean isCollectingAlgae() {
        return false; //TODO: Implement for when an algae should enter the robot
    }

    private static void updateEjection() {
        if (HELD_CORAL_INDEX != null && isEjectingCoral() && CAN_EJECT_CORAL) {
            final SimulatedGamePiece heldCoral = CORAL_ON_FIELD.get(HELD_CORAL_INDEX);
            heldCoral.release();
            HELD_CORAL_INDEX = null;
        }
        if (HELD_ALGAE_INDEX != null && isEjectingAlgae() && CAN_EJECT_ALGAE) {
            final SimulatedGamePiece heldAlgae = ALGAE_ON_FIELD.get(HELD_ALGAE_INDEX);
            heldAlgae.release();
            HELD_ALGAE_INDEX = null;
        }
    }

    private static boolean isEjectingCoral() {
        return RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.EJECT);//TODO: Implement for when a coral should exit the robot
    }

    private static boolean isEjectingAlgae() {
        return false;//TODO: Implement for when an algae should exit the robot
    }

    /**
     * Updates the position of the held game pieces so that they stay inside the robot.
     */
    private static void updateHeldGamePiecePoses() {
        final Pose3d
                robotRelativeHeldCoralPosition = RobotContainer.CORAL_INTAKE.calculateCollectedCoralPose(),
                robotRelativeHeldAlgaePosition = new Pose3d();
        updateHeldGamePiecePose(robotRelativeHeldCoralPosition, CORAL_ON_FIELD, HELD_CORAL_INDEX);
        updateHeldGamePiecePose(robotRelativeHeldAlgaePosition, ALGAE_ON_FIELD, HELD_ALGAE_INDEX);
    }

    private static void updateHeldGamePiecePose(Pose3d robotRelativeHeldGamePiecePose, ArrayList<SimulatedGamePiece> gamePieceList, Integer heldGamePieceIndex) {
        if (heldGamePieceIndex == null)
            return;

        final SimulatedGamePiece heldGamePiece = gamePieceList.get(heldGamePieceIndex);
        heldGamePiece.updatePose(calculateHeldGamePieceFieldRelativePose(robotRelativeHeldGamePiecePose));
    }

    /**
     * Calculate the position of the held game piece relative to the field.
     *
     * @param robotRelativeHeldGamePiecePosition the position of the held game piece relative to the robot
     * @return the position of the held game piece relative to the field
     */
    private static Pose3d calculateHeldGamePieceFieldRelativePose(Pose3d robotRelativeHeldGamePiecePosition) {
        final Pose3d robotPose3d = new Pose3d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
        return robotPose3d.plus(toTransform(robotRelativeHeldGamePiecePosition));
    }

    /**
     * Changes a Pose3d into a Transform3d.
     *
     * @param pose the target Pose3d
     * @return the Transform3d
     */
    private static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    private static Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<SimulatedGamePiece> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = gamePieces.get(i).getPose();
        return poses;
    }
}
