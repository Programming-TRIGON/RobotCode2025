package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulatedGamePiece {
    private final double originPointHeightOffGroundMeters;
    private Pose3d fieldRelativePose;

    public SimulatedGamePiece(Pose3d startingPose, SimulatedGamePieceConstants.GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        originPointHeightOffGroundMeters = gamePieceType.originPointHeightOffGroundMeters;
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public boolean isTouchingGround() {
        return fieldRelativePose.getTranslation().getZ() <= originPointHeightOffGroundMeters;
    }

    public double getDistanceMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }
}
