package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulatedGamePiece {
    private final double originPointHeightOffGroundMeters;
    private Pose3d fieldRelativePose;

    public static SimulatedGamePiece generateCoral(Pose3d startingPose) {
        return new SimulatedGamePiece(startingPose, SimulatedGamePieceConstants.CORAL_ORIGIN_POINT_HEIGHT_OFF_GROUND_METERS);
    }

    public static SimulatedGamePiece generateAlgae(Pose3d startingPose) {
        return new SimulatedGamePiece(startingPose, SimulatedGamePieceConstants.ALGAE_ORIGIN_POINT_HEIGHT_OFF_GROUND_METERS);
    }

    public SimulatedGamePiece(Pose3d startingPose, double originPointHeightOffGroundMeters) {
        fieldRelativePose = startingPose;
        this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public boolean isGrounded() {
        return fieldRelativePose.getTranslation().getZ() <= originPointHeightOffGroundMeters;
    }

    public double getDistanceMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }
}
