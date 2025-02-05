package frc.trigon.robot.subsystems.climber.climbervisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.Conversions;

public class ClimberVisualization {
    public void update(double currentPositionRotations, double targetPositionRotations) {
        final double currentStringLengthMeters = calculateStringLengthMeters(currentPositionRotations);
        final double targetStringLengthMeters = calculateStringLengthMeters(targetPositionRotations);
        final Rotation2d currentArmAngle = calculateArmAngle(currentStringLengthMeters);
        final Rotation2d targetArmAngle = calculateArmAngle(targetStringLengthMeters);
        final Rotation2d currentStringAngle = calculateStringAngle(currentArmAngle, currentStringLengthMeters);
        final Rotation2d targetStringAngle = calculateStringAngle(targetArmAngle, targetStringLengthMeters);

        setCurrentMechanism2dState(currentArmAngle, currentStringAngle, currentStringLengthMeters);
        setTargetMechanism2dState(targetArmAngle, targetStringAngle, targetStringLengthMeters);
        log();
    }

    private void setCurrentMechanism2dState(Rotation2d armAngle, Rotation2d stringAngle, double stringLengthMeters) {
        ClimberVisualizationConstants.CURRENT_ARM_POSITION_LIGAMENT.setAngle(armAngle.getDegrees());
        ClimberVisualizationConstants.CURRENT_STRING_POSITION_LIGAMENT.setAngle(stringAngle.getDegrees());
        ClimberVisualizationConstants.CURRENT_STRING_POSITION_LIGAMENT.setLength(stringLengthMeters);
    }

    private void setTargetMechanism2dState(Rotation2d targetArmAngle, Rotation2d targetStringAngle, double targetStringLengthMeters) {
        ClimberVisualizationConstants.TARGET_ARM_POSITION_LIGAMENT.setAngle(targetArmAngle.getDegrees());
        ClimberVisualizationConstants.TARGET_STRING_POSITION_LIGAMENT.setAngle(targetStringAngle.getDegrees());
        ClimberVisualizationConstants.TARGET_STRING_POSITION_LIGAMENT.setLength(targetStringLengthMeters);
    }

    private void log() {
        Logger.recordOutput(ClimberVisualizationConstants.MECHANISM_KEY, ClimberVisualizationConstants.MECHANISM);
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
        return Rotation2d.fromDegrees(180).minus(angle);
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberVisualizationConstants.DRUM_DIAMETER_METERS);
    }
}