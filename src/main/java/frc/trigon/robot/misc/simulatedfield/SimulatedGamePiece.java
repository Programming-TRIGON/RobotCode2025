package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulatedGamePiece {
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private Pose3d fieldRelativePose;
    private boolean isScored = false;

    public SimulatedGamePiece(Pose3d startingPose, SimulatedGamePieceConstants.GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        this.gamePieceType = gamePieceType;
    }

    public SimulatedGamePieceConstants.GamePieceType getGamePieceType() {
        return gamePieceType;
    }

    public void update() {
        checkScoring();
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

    public boolean isScored() {
        return isScored;
    }

    public void setScored(boolean scored) {
        isScored = scored;
    }

    private void checkScoring() {

    }
}
