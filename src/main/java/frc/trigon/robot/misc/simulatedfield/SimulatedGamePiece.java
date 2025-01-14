package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulatedGamePiece {
    private Pose3d fieldRelativePose;
    private Pose3d poseAtRelease;
    private double timestampAtRelease = 0;
    private boolean isTouchingGround = true;
    protected SimulatedGamePieceConstants.GamePieceType gamePieceType;
    protected boolean isScored = false;

    public SimulatedGamePiece(Pose3d startingPose, SimulatedGamePieceConstants.GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        this.gamePieceType = gamePieceType;
    }

    public void updatePeriodically() {
        if (!isScored && !isTouchingGround)
            applyGravity();
    }

    public void release() {
        poseAtRelease = fieldRelativePose;
        timestampAtRelease = Timer.getTimestamp();

        updateIsTouchingGround();
        checkScored();
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public double getDistanceMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }

    private void applyGravity() {
        if (poseAtRelease == null)
            return;
        final double timeSinceEject = Timer.getTimestamp() - timestampAtRelease;
        this.fieldRelativePose = new Pose3d(
                fieldRelativePose.getTranslation().getX(),
                fieldRelativePose.getTranslation().getY(),
                poseAtRelease.getTranslation().getZ() - SimulatedGamePieceConstants.G_FORCE / 2 * timeSinceEject * timeSinceEject,
                poseAtRelease.getRotation()
        );
        updateIsTouchingGround();
    }

    private void checkScored() {
        if (!isScored)
            SimulationScoringHandler.checkGamePieceScored(this);
    }

    private void updateIsTouchingGround() {
        if (fieldRelativePose.getZ() < gamePieceType.originPointHeightOffGroundMeters) {
            isTouchingGround = true;
            fieldRelativePose = new Pose3d(
                    fieldRelativePose.getTranslation().getX(),
                    fieldRelativePose.getTranslation().getY(),
                    gamePieceType.originPointHeightOffGroundMeters,
                    new Rotation3d(
                            fieldRelativePose.getRotation().getX(),
                            0,
                            fieldRelativePose.getRotation().getZ()
                    )
            );
            return;
        }
        isTouchingGround = false;
    }
}
