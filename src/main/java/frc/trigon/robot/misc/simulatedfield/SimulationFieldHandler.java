package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;
import frc.trigon.robot.subsystems.algaeintake.AlgaeIntakeConstants;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
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
        if (isCollectingCoralFromSource() && HELD_CORAL_INDEX == null && (new Transform2d(robotPose.toPose2d(), SimulatedGamePieceConstants.LEFT_FEEDER_POSITION).getTranslation().getNorm() < SimulatedGamePieceConstants.CORAL_FEEDER_INTAKE_TOLERANCE_METERS || new Transform2d(robotPose.toPose2d(), SimulatedGamePieceConstants.RIGHT_FEEDER_POSITION).getTranslation().getNorm() < SimulatedGamePieceConstants.CORAL_FEEDER_INTAKE_TOLERANCE_METERS)) {
            CORAL_ON_FIELD.add(new SimulatedGamePiece(new Pose3d(), SimulatedGamePieceConstants.GamePieceType.CORAL));
            HELD_CORAL_INDEX = CORAL_ON_FIELD.size() - 1;
        }
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

    private static boolean isCollectingCoralFromSource() {
        return RobotContainer.GRIPPER.atState(GripperConstants.GripperState.COLLECT_FROM_FEEDER) && RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.COLLECT_FROM_FEEDER);
    }

    private static boolean isCollectingAlgae() {
        return RobotContainer.ALGAE_INTAKE.atState(AlgaeIntakeConstants.AlgaeIntakeState.COLLECT);
    }

    private static void updateEjection() {
        if (HELD_CORAL_INDEX != null && CAN_EJECT_CORAL) {
            final SimulatedGamePiece heldCoral = CORAL_ON_FIELD.get(HELD_CORAL_INDEX);
            if (isIntakeEjectingCoral()) {
                heldCoral.release();
                HELD_CORAL_INDEX = null;
            }
            if (isEjectingCoral()) {
                final Pose3d robotPose = new Pose3d(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose());
                final Pose3d robotRelativeGripperReleasePose = RobotContainer.GRIPPER.calculateCoralReleasePoint();
                final Translation3d robotRelativeReleaseVelocity = RobotContainer.GRIPPER.getRobotRelativeExitVelocity();
                final ChassisSpeeds robotSpeeds = RobotContainer.SWERVE.getSelfRelativeVelocity();
                final Translation3d robotVelocity = new Translation3d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, 0);
                heldCoral.release(robotPose.plus(toTransform(robotRelativeGripperReleasePose)), robotVelocity.plus(robotRelativeReleaseVelocity).rotateBy(new Rotation3d(RobotContainer.SWERVE.getHeading())));
                HELD_CORAL_INDEX = null;
            }
        }
        if (HELD_ALGAE_INDEX != null && isEjectingAlgae() && CAN_EJECT_ALGAE) {
            final SimulatedGamePiece heldAlgae = ALGAE_ON_FIELD.get(HELD_ALGAE_INDEX);
            heldAlgae.release();
            HELD_ALGAE_INDEX = null;
        }
    }

    private static boolean isEjectingCoral() {
        if (RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.SCORE_L1) && RobotContainer.GRIPPER.atState(GripperConstants.GripperState.SCORE_L1))
            return true;
        if (RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.SCORE_L2) && RobotContainer.GRIPPER.atState(GripperConstants.GripperState.SCORE_L3_OR_L2))
            return true;
        if (RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.SCORE_L3) && RobotContainer.GRIPPER.atState(GripperConstants.GripperState.SCORE_L3_OR_L2))
            return true;
        return RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.SCORE_L4) && RobotContainer.GRIPPER.atState(GripperConstants.GripperState.SCORE_L4);
    }

    private static boolean isIntakeEjectingCoral() {
        return RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.EJECT);
    }

    private static boolean isEjectingAlgae() {
        return RobotContainer.ALGAE_INTAKE.atState(AlgaeIntakeConstants.AlgaeIntakeState.EJECT) ||
                RobotContainer.ALGAE_INTAKE.atState(AlgaeIntakeConstants.AlgaeIntakeState.FEED_PROCESSOR);
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