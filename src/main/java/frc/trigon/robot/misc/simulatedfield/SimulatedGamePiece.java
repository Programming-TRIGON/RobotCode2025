package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;

public class SimulatedGamePiece {
    private final GamePieceType gamePieceType;
    private Pose3d fieldRelativePose;

    public SimulatedGamePiece(Pose3d startingPose, GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        this.gamePieceType = gamePieceType;
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public boolean isGrounded() {
        return fieldRelativePose.getTranslation().getZ() <= gamePieceType.originPointHeightOffGroundMeters;
    }

    public double getDistanceMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }

    public enum GamePieceType {
        CORAL(0.4),
        ALGAE(0.15);

        private final double originPointHeightOffGroundMeters;

        GamePieceType(double originPointHeightOffGroundMeters) {
            this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
        }
    }
}
