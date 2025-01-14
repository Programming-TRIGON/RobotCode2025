package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulatedGamePiece {
    final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private Pose3d fieldRelativePose;
    private Pose3d poseAtRelease;
    private double timeAtRelease = 0;
    boolean isScored = false, isTouchingGround = true;

    public SimulatedGamePiece(Pose3d startingPose, SimulatedGamePieceConstants.GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        this.gamePieceType = gamePieceType;
    }

    public void update() {
        if (!isScored && !isTouchingGround())
            fallWithGravity();
    }

    public void release() {
        updateIsTouchingGround();
        checkScored();
        if (!isScored)
            fieldRelativePose = new Pose3d(
                    fieldRelativePose.getTranslation(),
                    new Rotation3d(
                            fieldRelativePose.getRotation().getX(),
                            0,
                            fieldRelativePose.getRotation().getZ()
                    )
            );

        poseAtRelease = fieldRelativePose;
        timeAtRelease = Timer.getTimestamp();
    }

    public void updatePose(Pose3d fieldRelativePose) {
        this.fieldRelativePose = fieldRelativePose;
    }

    public Pose3d getPose() {
        return fieldRelativePose;
    }

    public boolean isTouchingGround() {
        return isTouchingGround;
    }

    public double getDistanceMeters(Pose3d pose) {
        return fieldRelativePose.minus(pose).getTranslation().getNorm();
    }

    private void fallWithGravity() {
        final double timeSinceEject = Timer.getTimestamp() - timeAtRelease;
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
                    fieldRelativePose.getRotation()
            );
            return;
        }
        isTouchingGround = false;
    }
}
