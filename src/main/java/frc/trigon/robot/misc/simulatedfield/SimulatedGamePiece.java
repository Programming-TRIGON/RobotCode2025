package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulatedGamePiece {
    private Pose3d fieldRelativePose;
    final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    boolean isScored = false;

    public SimulatedGamePiece(Pose3d startingPose, SimulatedGamePieceConstants.GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        this.gamePieceType = gamePieceType;
    }

    public void checkScored() {
        if (!isScored)
            SimulationScoringHandler.checkGamePieceScored(this);
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public boolean isTouchingGround() {
        return fieldRelativePose.getTranslation().getZ() <= gamePieceType.originPointHeightOffGroundMeters;
    }

    public double getDistanceMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }
}
