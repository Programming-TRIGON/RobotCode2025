package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * Handles the simulation of game pieces.
 */
public class SimulationFieldHandler {
    private static boolean
            IS_HOLDING_CORAL = false,
            IS_HOLDING_ALGAE = false;
    private static final ArrayList<SimulatedGamePiece>
            CORAL_ON_FIELD = FieldConstants.CORAL_ON_FIELD,
            ALGAE_ON_FIELD = FieldConstants.ALGAE_ON_FIELD;
    private static boolean CAN_EJECT = false;
    private static final double PICKUP_TOLERANCE_METERS = 0.3;
    private static Integer
            HELD_CORAL_INDEX = null,
            HELD_ALGAE_INDEX = null;

    public static void update() {
        updateGamePieces();
        logGamePieces();
    }

    /**
     * Updates the state of all game pieces.
     */
    private static void updateGamePieces() {
        updateCoralCollection();
        updateCoralEjection();
        updateHeldCoralPose();
    }

    public static ArrayList<SimulatedGamePiece> getSimulatedCoral() {
        return CORAL_ON_FIELD;
    }

    public static ArrayList<SimulatedGamePiece> getSimulatedAlgae() {
        return ALGAE_ON_FIELD;
    }

    public static boolean isHoldingCoral() {
        return IS_HOLDING_CORAL;
    }

    public static boolean isHoldingAlgae() {
        return IS_HOLDING_ALGAE;
    }

    private static SimulatedCoral[] generateCoral(Pose3d[] startingPoses) {
        final SimulatedCoral[] GAME_PIECES = new SimulatedCoral[120];
        for (int i = 0; i < startingPoses.length; i++)
            GAME_PIECES[i] = new SimulatedCoral(startingPoses[i]);
        return GAME_PIECES;
    }

    private static SimulatedAlgae[] generateAlgae(Pose3d[] startingPoses) {
        final SimulatedAlgae[] GAME_PIECES = new SimulatedAlgae[18];
        for (int i = 0; i < startingPoses.length; i++)
            GAME_PIECES[i] = new SimulatedAlgae(startingPoses[i]);
        return GAME_PIECES;
    }

    /**
     * Handles when a coral should collect by checking the state of the necessary subsystems of the robot and the position of the robot relative to the coral.
     * Also regenerates picked up coral.
     */
    private static void updateCoralCollection() {
        if (HELD_CORAL_INDEX != null || !isCollectingCoral())
            return;
        final Pose2d robotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        for (int i = 0; i < CORAL_ON_FIELD.size(); i++) {
            final SimulatedGamePiece coral = CORAL_ON_FIELD.get(i);
            if (coral.getDistanceMeters(new Pose3d(robotPose)) <= PICKUP_TOLERANCE_METERS && coral.isGrounded()) {
                HELD_CORAL_INDEX = i;
                IS_HOLDING_CORAL = true;
                CAN_EJECT = true;
                return;
            }
        }
    }

    /**
     * Handles when a held coral should eject from the robot by checking the states of the necessary subsystems.
     */
    private static void updateCoralEjection() {
        if (HELD_CORAL_INDEX == null || !isEjectingCoral() || !CAN_EJECT)
            return;
        CAN_EJECT = false;
        GeneralCommands.getDelayedCommand(0.04, () -> {
            HELD_CORAL_INDEX = null;
            IS_HOLDING_CORAL = false;
        }).schedule();
    }

    /**
     * Updates the position of the held coral so that it stays inside the robot.
     */
    private static void updateHeldCoralPose() {
        if (HELD_CORAL_INDEX == null)
            return;
        CORAL_ON_FIELD.get(HELD_CORAL_INDEX).updatePose(calculateHeldCoralPose(RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose()));
    }

    private static boolean isCollectingCoral() {
        return OperatorConstants.OPERATOR_CONTROLLER.c().getAsBoolean();//RobotContainer.CORAL_INTAKE.atState(CoralIntakeConstants.CoralIntakeState.COLLECT);
    }

    private static boolean isEjectingCoral() {
        return OperatorConstants.OPERATOR_CONTROLLER.e().getAsBoolean();//TODO: Implement for when a coral should exit the robot
    }

    /**
     * Calculate the position of the held coral relative to the field.
     *
     * @param robotPose the position of the robot on the field
     * @return the position of the coral relative to the field
     */
    private static Pose3d calculateHeldCoralPose(Pose2d robotPose) {
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
    private static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Logs the position of all the game pieces on the field and in the robot.
     */
    private static void logGamePieces() {
        Logger.recordOutput("Poses/GamePieces/Coral", mapSimulatedGamePieceListToPoseArray(CORAL_ON_FIELD));
        Logger.recordOutput("Poses/GamePieces/Algae", mapSimulatedGamePieceListToPoseArray(ALGAE_ON_FIELD));
    }

    private static Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<SimulatedGamePiece> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = gamePieces.get(i).getPose();
        }
        return poses;
    }
}
