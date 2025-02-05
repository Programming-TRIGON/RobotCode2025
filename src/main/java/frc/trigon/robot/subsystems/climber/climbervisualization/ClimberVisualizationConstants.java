package frc.trigon.robot.subsystems.climber.climbervisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimberVisualizationConstants {
    static final Translation3d CLIMBER_ORIGIN_POINT = new Translation3d(); //TODO: Set this to the actual origin point
    static final double DRUM_DIAMETER_METERS = 0.1016;
    static final double JOINT_TO_STRING_CONNECTION_LENGTH_METERS = 0.227;
    static final double JOINT_TO_DRUM_LENGTH_METERS = 0.534;
    static final double RETRACTED_STRING_LENGTH_METERS = 0.3145;
    static final Rotation2d
            RETRACTED_ARM_ANGLE = Rotation2d.fromDegrees(14),
            RETRACTED_STRING_ANGLE = Rotation2d.fromDegrees(9.34);
    static final String
            NAME = "Climber",
            MECHANISM_KEY = "Mechanisms/" + NAME;
    static final double MECHANISM_LINE_WIDTH = 5;
    static final LoggedMechanism2d MECHANISM = new LoggedMechanism2d(
            JOINT_TO_DRUM_LENGTH_METERS * 2,
            JOINT_TO_DRUM_LENGTH_METERS * 2
    );
    static final LoggedMechanismRoot2d
            STRING_LIGAMENT_ROOT = MECHANISM.getRoot("StringLigament", JOINT_TO_DRUM_LENGTH_METERS / 2, 0),
            ARM_LIGAMENT_ROOT = MECHANISM.getRoot("DrumLigament", (JOINT_TO_DRUM_LENGTH_METERS / 2) + JOINT_TO_DRUM_LENGTH_METERS, 0);
    static final LoggedMechanismLigament2d
            CURRENT_STRING_POSITION_LIGAMENT = STRING_LIGAMENT_ROOT.append(new LoggedMechanismLigament2d("ZCurrentStringLigament", RETRACTED_STRING_LENGTH_METERS, RETRACTED_STRING_ANGLE.getDegrees(), MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGreen))),
            TARGET_STRING_POSITION_LIGAMENT = STRING_LIGAMENT_ROOT.append(new LoggedMechanismLigament2d("TargetStringLigament", RETRACTED_STRING_LENGTH_METERS, RETRACTED_STRING_ANGLE.getDegrees(), MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray))),
            CURRENT_ARM_POSITION_LIGAMENT = ARM_LIGAMENT_ROOT.append(new LoggedMechanismLigament2d("ZCurrentArmLigament", JOINT_TO_STRING_CONNECTION_LENGTH_METERS, RETRACTED_ARM_ANGLE.getDegrees(), MECHANISM_LINE_WIDTH, new Color8Bit(Color.kLightGreen))),
            TARGET_ARM_POSITION_LIGAMENT = ARM_LIGAMENT_ROOT.append(new LoggedMechanismLigament2d("TargetArmLigament", JOINT_TO_STRING_CONNECTION_LENGTH_METERS, RETRACTED_ARM_ANGLE.getDegrees(), MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));
}