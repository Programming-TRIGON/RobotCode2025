package frc.trigon.robot.subsystems.climber.climbervisualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.Conversions;

public class ClimberVisualization {
    /**
     * Updates the 2d mechanism's current climber arm angle, string angle, and string length. Then calculates the 3d Advantage Scope poses, and logs 2d mechanism and 3d poses.
     *
     * @param currentPositionRotations the current position of the climber
     * @param targetPositionRotations  the target position of the climber
     */
    public void update(double currentPositionRotations, double targetPositionRotations) {
        final double currentStringLengthMeters = calculateStringLengthMeters(currentPositionRotations);
        final double targetStringLengthMeters = calculateStringLengthMeters(targetPositionRotations);
        final Rotation2d currentArmAngle = calculateArmAngle(currentStringLengthMeters);
        final Rotation2d targetArmAngle = calculateArmAngle(targetStringLengthMeters);
        final Rotation2d currentStringAngle = calculateStringAngle(currentArmAngle, currentStringLengthMeters);
        final Rotation2d targetStringAngle = calculateStringAngle(targetArmAngle, targetStringLengthMeters);

        setCurrentMechanism2dState(currentArmAngle, currentStringAngle, currentStringLengthMeters);
        setTargetMechanism2dState(targetArmAngle, targetStringAngle, targetStringLengthMeters);
        log(currentArmAngle);
    }

    /**
     * Sets the current state of the 2d mechanism.
     *
     * @param currentArmAngle           the current arm angle
     * @param currentStringAngle        the current string angle
     * @param currentStringLengthMeters the current string length
     */
    private void setCurrentMechanism2dState(Rotation2d currentArmAngle, Rotation2d currentStringAngle, double currentStringLengthMeters) {
        ClimberVisualizationConstants.CURRENT_ARM_POSITION_LIGAMENT.setAngle(flipAngle(currentArmAngle).getDegrees());
        ClimberVisualizationConstants.CURRENT_STRING_POSITION_LIGAMENT.setAngle(currentStringAngle.getDegrees());
        ClimberVisualizationConstants.CURRENT_STRING_POSITION_LIGAMENT.setLength(currentStringLengthMeters);
    }

    /**
     * Sets the target state of the 2d mechanism.
     *
     * @param targetArmAngle           the target arm angle
     * @param targetStringAngle        the target string angle
     * @param targetStringLengthMeters the target string length
     */
    private void setTargetMechanism2dState(Rotation2d targetArmAngle, Rotation2d targetStringAngle, double targetStringLengthMeters) {
        ClimberVisualizationConstants.TARGET_ARM_POSITION_LIGAMENT.setAngle(flipAngle(targetArmAngle).getDegrees());
        ClimberVisualizationConstants.TARGET_STRING_POSITION_LIGAMENT.setAngle(targetStringAngle.getDegrees());
        ClimberVisualizationConstants.TARGET_STRING_POSITION_LIGAMENT.setLength(targetStringLengthMeters);
    }

    /**
     * Logs the 2d mechanism and 3d advantage scope poses.
     *
     * @param currentArmAngle the current arm angle
     */
    private void log(Rotation2d currentArmAngle) {
        Logger.recordOutput("Mechanisms/Climber", ClimberVisualizationConstants.MECHANISM);
        Logger.recordOutput("Poses/Components/Climber", calculateArmPose(currentArmAngle));
    }

    private Pose3d calculateArmPose(Rotation2d currentArmAngle) {
        return new Pose3d(
                ClimberVisualizationConstants.CLIMBER_ORIGIN_POINT,
                new Rotation3d(0, currentArmAngle.getRadians(), 0)
        );
    }

    private Rotation2d calculateStringAngle(Rotation2d armAngle, double stringLengthMeters) {
        final double numerator = armAngle.getSin() * ClimberVisualizationConstants.JOINT_TO_STRING_CONNECTION_LENGTH_METERS;
        final double division = numerator / stringLengthMeters;
        final double stringAngleRadians = Math.asin(division);
        return Rotation2d.fromRadians(stringAngleRadians);
    }

    private Rotation2d calculateArmAngle(double stringLengthMeters) {
        final double numerator =
                (ClimberVisualizationConstants.JOINT_TO_STRING_CONNECTION_LENGTH_METERS * ClimberVisualizationConstants.JOINT_TO_STRING_CONNECTION_LENGTH_METERS) +
                        (ClimberVisualizationConstants.JOINT_TO_DRUM_LENGTH_METERS * ClimberVisualizationConstants.JOINT_TO_DRUM_LENGTH_METERS) -
                        (stringLengthMeters * stringLengthMeters);
        final double denominator = 2 * ClimberVisualizationConstants.JOINT_TO_STRING_CONNECTION_LENGTH_METERS * ClimberVisualizationConstants.JOINT_TO_DRUM_LENGTH_METERS;
        final double armAngleRadians = Math.acos(numerator / denominator);
        return Rotation2d.fromRadians(armAngleRadians);
    }

    private double calculateStringLengthMeters(double positionRotations) {
        return toMeters(positionRotations) + ClimberVisualizationConstants.RETRACTED_STRING_LENGTH_METERS;
    }

    /**
     * Flips the angle.
     *
     * @param angle the angle to flip
     * @return the flipped angle
     */
    private Rotation2d flipAngle(Rotation2d angle) {
        return Rotation2d.fromDegrees(180 - angle.getDegrees());
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberVisualizationConstants.DRUM_DIAMETER_METERS);
    }
}