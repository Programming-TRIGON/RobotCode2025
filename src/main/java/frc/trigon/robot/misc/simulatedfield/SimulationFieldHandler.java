package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
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
    private static Integer
            HELD_CORAL_INDEX = 0,
            HELD_ALGAE_INDEX = null;
    private static boolean IS_CORAL_IN_GRIPPER = true;

    public static ArrayList<SimulatedGamePiece> getSimulatedCoral() {
        return CORAL_ON_FIELD;
    }

    public static ArrayList<SimulatedGamePiece> getSimulatedAlgae() {
        return ALGAE_ON_FIELD;
    }

    public static boolean isHoldingCoral() {
        return HELD_CORAL_INDEX != null;
    }

    public static boolean isCoralInGripper() {
        return IS_CORAL_IN_GRIPPER;
    }

    public static boolean isHoldingAlgae() {
        return HELD_ALGAE_INDEX != null;
    }

    public static void update() {
        updateGamePieces();
        logGamePieces();
    }

    /**
     * Updates the state of all game pieces.
     */
    private static void updateGamePieces() {
        updateGamePiecesPeriodically();
        updateCollection();
        updateCoralLoading();
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
            coral.updatePeriodically(HELD_CORAL_INDEX != null && HELD_CORAL_INDEX == CORAL_ON_FIELD.indexOf(coral));
        for (SimulatedGamePiece algae : ALGAE_ON_FIELD)
            algae.updatePeriodically(HELD_CORAL_INDEX != null && HELD_CORAL_INDEX == CORAL_ON_FIELD.indexOf(algae));
    }

    private static void updateCollection() {
        final Pose3d robotPose = new Pose3d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
        final Pose3d
                coralCollectionPose = robotPose.plus(toTransform(RobotContainer.CORAL_INTAKE.calculateCoralCollectionPose())),
                algaeCollectionPose = robotPose.plus(toTransform(new Pose3d()));

        updateCoralFeederCollection(robotPose);

        if (isCollectingCoral() && HELD_CORAL_INDEX == null)
            HELD_CORAL_INDEX = getIndexOfCollectedGamePiece(coralCollectionPose, CORAL_ON_FIELD, SimulatedGamePieceConstants.CORAL_INTAKE_TOLERANCE_METERS);

        if (isCollectingAlgae() && HELD_ALGAE_INDEX == null)
            HELD_ALGAE_INDEX = getIndexOfCollectedGamePiece(algaeCollectionPose, ALGAE_ON_FIELD, SimulatedGamePieceConstants.ALGAE_INTAKE_TOLERANCE_METERS);
    }

    private static void updateCoralFeederCollection(Pose3d robotPose) {
        final double
                distanceFromLeftFeeder = robotPose.toPose2d().getTranslation().getDistance(SimulatedGamePieceConstants.LEFT_FEEDER_POSITION.get()),
                distanceFromRightFeeder = robotPose.toPose2d().getTranslation().getDistance(SimulatedGamePieceConstants.RIGHT_FEEDER_POSITION.get());
        final double distanceFromBestFeederMeters = Math.min(distanceFromLeftFeeder, distanceFromRightFeeder);

        if (isCollectingCoral() && HELD_CORAL_INDEX == null &&
                distanceFromBestFeederMeters < SimulatedGamePieceConstants.CORAL_FEEDER_INTAKE_TOLERANCE_METERS) {
            CORAL_ON_FIELD.add(new SimulatedGamePiece(new Pose3d(), SimulatedGamePieceConstants.GamePieceType.CORAL));
            HELD_CORAL_INDEX = CORAL_ON_FIELD.size() - 1;
        }
    }

    private static void updateCoralLoading() {
        if (!RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER))
            return;

        IS_CORAL_IN_GRIPPER = true;
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
            if (gamePiece.getDistanceFromPoseMeters(collectionPose) <= intakeTolerance)
                return gamePieceList.indexOf(gamePiece);
        return null;
    }

    private static boolean isCollectingCoral() {
        return RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR);
    }

    private static boolean isCollectingCoralFromSource() {
        return false;//TODO: Implement
    }

    private static boolean isCollectingAlgae() {
        return false;//TODO: Implement
    }

    private static void updateEjection() {
        if (HELD_CORAL_INDEX != null)
            updateCoralEjection();

        if (HELD_ALGAE_INDEX != null && isEjectingAlgae()) {
            final SimulatedGamePiece heldAlgae = ALGAE_ON_FIELD.get(HELD_ALGAE_INDEX);
            heldAlgae.release();
            HELD_ALGAE_INDEX = null;
        }
    }

    private static void updateCoralEjection() {
        final SimulatedGamePiece heldCoral = CORAL_ON_FIELD.get(HELD_CORAL_INDEX);

        if (isIntakeEjectingCoral()) {
            heldCoral.release();
            HELD_CORAL_INDEX = null;
        }

        if (isGripperEjectingCoral())
            ejectCoralFromGripper(heldCoral);
    }

    private static void ejectCoralFromGripper(SimulatedGamePiece ejectedCoral) {
        final Pose3d robotPose = new Pose3d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
        final Pose3d robotRelativeGripperReleasePose = RobotContainer.GRIPPER.calculateCoralReleasePoint();
        final Translation3d robotRelativeReleaseVelocity = RobotContainer.GRIPPER.getRobotRelativeExitVelocity();
        final ChassisSpeeds swerveWheelSpeeds = RobotContainer.SWERVE.getSelfRelativeVelocity();
        final Translation3d robotSelfRelativeVelocity = new Translation3d(swerveWheelSpeeds.vxMetersPerSecond, swerveWheelSpeeds.vyMetersPerSecond, 0);

        ejectedCoral.updatePose(robotPose.transformBy(toTransform(robotRelativeGripperReleasePose)));
        ejectedCoral.release(robotSelfRelativeVelocity.plus(robotRelativeReleaseVelocity).rotateBy(new Rotation3d(RobotContainer.SWERVE.getHeading())));

        HELD_CORAL_INDEX = null;
        IS_CORAL_IN_GRIPPER = false;
    }

    private static boolean isGripperEjectingCoral() {
        return RobotContainer.GRIPPER.isEjecting() && IS_CORAL_IN_GRIPPER;
    }

    private static boolean isIntakeEjectingCoral() {
        return RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.EJECT) && !IS_CORAL_IN_GRIPPER;
    }

    private static boolean isEjectingAlgae() {
        return false;//TODO: Implement
    }

    /**
     * Updates the position of the held game pieces so that they stay inside the robot.
     */
    private static void updateHeldGamePiecePoses() {
        final Pose3d
                robotRelativeHeldCoralPosition = IS_CORAL_IN_GRIPPER ? RobotContainer.GRIPPER.calculateHeldCoralVisualizationPose() : RobotContainer.CORAL_INTAKE.calculateCollectedCoralPose(),
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
     * Converts a Pose3d into a Transform3d.
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