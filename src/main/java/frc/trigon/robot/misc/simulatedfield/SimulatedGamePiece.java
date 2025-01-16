package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulatedGamePiece {
    protected SimulatedGamePieceConstants.GamePieceType gamePieceType;
    protected boolean isScored = false;
    private Pose3d fieldRelativePose;
    private Pose3d poseAtRelease;
    private Translation2d velocityAtRelease = new Translation2d();
    private double timestampAtRelease = 0;
    private boolean isTouchingGround = true;

    public SimulatedGamePiece(Pose3d startingPose, SimulatedGamePieceConstants.GamePieceType gamePieceType) {
        fieldRelativePose = startingPose;
        this.gamePieceType = gamePieceType;
    }

    public void updatePeriodically() {
        checkScored();
        if (!isScored && !isTouchingGround)
            applyGravity();
    }

    public void release(Pose3d releasePose, Translation2d fieldRelativeReleaseVelocity) {
        fieldRelativePose = releasePose;
        velocityAtRelease = fieldRelativeReleaseVelocity;
        release();
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
        this.fieldRelativePose = new Pose3d(poseAtRelease.getTranslation(), new Rotation3d()).transformBy(calculatePoseTransform(timeSinceEject));

        updateIsTouchingGround();
    }

    private Transform3d calculatePoseTransform(double elapsedTime) {
        return new Transform3d(
                velocityAtRelease.getX() * elapsedTime,
                velocityAtRelease.getY() * elapsedTime,
                -SimulatedGamePieceConstants.G_FORCE / 2 * elapsedTime * elapsedTime,
                poseAtRelease.getRotation()
        );
    }

    private void checkScored() {
        if (!isScored)
            SimulationScoringHandler.checkGamePieceScored(this);
    }

    private void updateIsTouchingGround() {
        if (fieldRelativePose.getZ() < gamePieceType.originPointHeightOffGroundMeters) {
            isTouchingGround = true;
            velocityAtRelease = new Translation2d();

            final Translation3d fieldRelativeTranslation = new Translation3d(
                    fieldRelativePose.getTranslation().getX(),
                    fieldRelativePose.getTranslation().getY(),
                    gamePieceType.originPointHeightOffGroundMeters
            );
            final Rotation3d fieldRelativeRotation = new Rotation3d(
                    fieldRelativePose.getRotation().getX(),
                    0,
                    fieldRelativePose.getRotation().getZ()
            );
            fieldRelativePose = new Pose3d(fieldRelativeTranslation, fieldRelativeRotation);
            return;
        }
        isTouchingGround = false;
    }
}
