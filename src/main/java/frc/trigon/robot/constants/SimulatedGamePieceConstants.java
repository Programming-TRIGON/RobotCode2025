package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePieceConstants {
    public static final double G_FORCE = 9.806;
    public static final double
            CORAL_INTAKE_TOLERANCE_METERS = 0.3,
            CORAL_FEEDER_INTAKE_TOLERANCE_METERS = 1,
            ALGAE_INTAKE_TOLERANCE_METERS = 0.4,
            CORAL_SCORING_TOLERANCE_METERS = 0.1,
            ALGAE_SCORING_TOLERANCE_METERS = 0.3;

    private static final double
            FIELD_WIDTH_METERS = FieldConstants.FIELD_WIDTH_METERS,
            FIELD_LENGTH_METERS = FieldConstants.FIELD_LENGTH_METERS;
    private static final Rotation3d
            CORAL_TO_VERTICAL_POSITION_ROTATION = new Rotation3d(0, -Math.PI / 2, 0),
            CORAL_TO_L2_AND_L3_ROTATION = new Rotation3d(0, Units.degreesToRadians(35), 0);

    public static final ArrayList<SimulatedGamePiece>
            CORAL_ON_FIELD = new ArrayList<>(List.of(
            new SimulatedGamePiece(new Pose3d(1.22, FIELD_WIDTH_METERS / 2, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION), GamePieceType.CORAL),
            new SimulatedGamePiece(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION), GamePieceType.CORAL),
            new SimulatedGamePiece(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION), GamePieceType.CORAL),
            new SimulatedGamePiece(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION), GamePieceType.CORAL),
            new SimulatedGamePiece(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION), GamePieceType.CORAL),
            new SimulatedGamePiece(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION), GamePieceType.CORAL)
    )),
            ALGAE_ON_FIELD = new ArrayList<>(List.of(
                    new SimulatedGamePiece(new Pose3d(1.22, FIELD_WIDTH_METERS / 2, 0.5, new Rotation3d()), GamePieceType.ALGAE),
                    new SimulatedGamePiece(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.5, new Rotation3d()), GamePieceType.ALGAE),
                    new SimulatedGamePiece(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.5, new Rotation3d()), GamePieceType.ALGAE),
                    new SimulatedGamePiece(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2, 0.5, new Rotation3d()), GamePieceType.ALGAE),
                    new SimulatedGamePiece(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.5, new Rotation3d()), GamePieceType.ALGAE),
                    new SimulatedGamePiece(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.5, new Rotation3d()), GamePieceType.ALGAE)
            ));

    private static final double
            L2_SCORE_HEIGHT_METERS = 0.70,
            L3_SCORE_HEIGHT_METERS = 1.1,
            L4_SCORE_HEIGHT_METERS = 1.78;
    private static final Rotation3d
            CORAL_2_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3 * 2),
            CORAL_4_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3),
            CORAL_8_OCLOCK_ROTATION = new Rotation3d(0, 0, -Math.PI / 3),
            CORAL_10_OCLOCK_ROTATION = new Rotation3d(0, 0, -Math.PI / 3 * 2),
            CORAL_12_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI);
    private static final Pose3d FIELD_CENTER = new Pose3d(-FIELD_LENGTH_METERS / 2, -FIELD_WIDTH_METERS / 2, 0, new Rotation3d()).rotateBy(new Rotation3d(0, 0, Math.PI));

    private static final Translation3d
            REEF_CENTER_TO_L1_VECTOR = new Translation3d(0.652, 0.1643, 0.46),
            REEF_CENTER_TO_L2_VECTOR = new Translation3d(0.652, 0.1643, 0.6983),
            REEF_CENTER_TO_L3_VECTOR = new Translation3d(0.652, 0.1643, 1.1101),
            REEF_CENTER_TO_L4_VECTOR = new Translation3d(0.7796, 0.1643, 1.7345);
    private static final Rotation3d
            REEF_TO_2_OCLOCK_RIGHT_ROTATION = new Rotation3d(0, 0, 0),
            REEF_TO_2_OCLOCK_LEFT_ROTATION = new Rotation3d(0, 0, Math.PI / 6),
            REEF_TO_4_OCLOCK_RIGHT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 2),
            REEF_TO_4_OCLOCK_LEFT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 3),
            REEF_TO_6_OCLOCK_RIGHT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 4),
            REEF_TO_6_OCLOCK_LEFT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 5),
            REEF_TO_8_OCLOCK_RIGHT_ROTATION = new Rotation3d(0, 0, Math.PI),
            REEF_TO_8_OCLOCK_LEFT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 7),
            REEF_TO_10_OCLOCK_RIGHT_ROTATION = new Rotation3d(0, 0, Math.PI * 8),
            REEF_TO_10_OCLOCK_LEFT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 9),
            REEF_TO_12_OCLOCK_RIGHT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 10),
            REEF_TO_12_OCLOCK_LEFT_ROTATION = new Rotation3d(0, 0, Math.PI / 6 * 11);
    public static final ArrayList<Pose3d> P = new ArrayList<>(List.of(
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_2_OCLOCK_RIGHT_ROTATION), REEF_TO_2_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_2_OCLOCK_LEFT_ROTATION), REEF_TO_2_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_4_OCLOCK_RIGHT_ROTATION), REEF_TO_4_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_4_OCLOCK_LEFT_ROTATION), REEF_TO_4_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_6_OCLOCK_RIGHT_ROTATION), REEF_TO_6_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_6_OCLOCK_LEFT_ROTATION), REEF_TO_6_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_8_OCLOCK_RIGHT_ROTATION), REEF_TO_8_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_8_OCLOCK_LEFT_ROTATION), REEF_TO_8_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_10_OCLOCK_RIGHT_ROTATION), REEF_TO_10_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_10_OCLOCK_LEFT_ROTATION), REEF_TO_10_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_12_OCLOCK_RIGHT_ROTATION), REEF_TO_12_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L1_VECTOR.rotateBy(REEF_TO_12_OCLOCK_LEFT_ROTATION), REEF_TO_12_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_2_OCLOCK_RIGHT_ROTATION), REEF_TO_2_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_2_OCLOCK_LEFT_ROTATION), REEF_TO_2_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_4_OCLOCK_RIGHT_ROTATION), REEF_TO_4_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_4_OCLOCK_LEFT_ROTATION), REEF_TO_4_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_6_OCLOCK_RIGHT_ROTATION), REEF_TO_6_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_6_OCLOCK_LEFT_ROTATION), REEF_TO_6_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_8_OCLOCK_RIGHT_ROTATION), REEF_TO_8_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_8_OCLOCK_LEFT_ROTATION), REEF_TO_8_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_10_OCLOCK_RIGHT_ROTATION), REEF_TO_10_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_10_OCLOCK_LEFT_ROTATION), REEF_TO_10_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_12_OCLOCK_RIGHT_ROTATION), REEF_TO_12_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L2_VECTOR.rotateBy(REEF_TO_12_OCLOCK_LEFT_ROTATION), REEF_TO_12_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_2_OCLOCK_RIGHT_ROTATION), REEF_TO_2_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_2_OCLOCK_LEFT_ROTATION), REEF_TO_2_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_4_OCLOCK_RIGHT_ROTATION), REEF_TO_4_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_4_OCLOCK_LEFT_ROTATION), REEF_TO_4_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_6_OCLOCK_RIGHT_ROTATION), REEF_TO_6_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_6_OCLOCK_LEFT_ROTATION), REEF_TO_6_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_8_OCLOCK_RIGHT_ROTATION), REEF_TO_8_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_8_OCLOCK_LEFT_ROTATION), REEF_TO_8_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_10_OCLOCK_RIGHT_ROTATION), REEF_TO_10_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_10_OCLOCK_LEFT_ROTATION), REEF_TO_10_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_12_OCLOCK_RIGHT_ROTATION), REEF_TO_12_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L3_VECTOR.rotateBy(REEF_TO_12_OCLOCK_LEFT_ROTATION), REEF_TO_12_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_2_OCLOCK_RIGHT_ROTATION), REEF_TO_2_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_2_OCLOCK_LEFT_ROTATION), REEF_TO_2_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_4_OCLOCK_RIGHT_ROTATION), REEF_TO_4_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_4_OCLOCK_LEFT_ROTATION), REEF_TO_4_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_6_OCLOCK_RIGHT_ROTATION), REEF_TO_6_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_6_OCLOCK_LEFT_ROTATION), REEF_TO_6_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_8_OCLOCK_RIGHT_ROTATION), REEF_TO_8_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_8_OCLOCK_LEFT_ROTATION), REEF_TO_8_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_10_OCLOCK_RIGHT_ROTATION), REEF_TO_10_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_10_OCLOCK_LEFT_ROTATION), REEF_TO_10_OCLOCK_LEFT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_12_OCLOCK_RIGHT_ROTATION), REEF_TO_12_OCLOCK_RIGHT_ROTATION)),
            new Pose3d(FieldConstants.CENTER_OF_REEF_POSE.get()).transformBy(new Transform3d(REEF_CENTER_TO_L4_VECTOR.rotateBy(REEF_TO_12_OCLOCK_LEFT_ROTATION), REEF_TO_12_OCLOCK_LEFT_ROTATION))
    ));

    public static final ArrayList<Pose3d>
            CORAL_SCORING_LOCATIONS = new ArrayList<>(List.of(
            new Pose3d(-4.938, -0.164, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.938, 0.164, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.753, -0.483, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_4_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.753, 0.483, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_8_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.469, -0.648, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_4_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.469, 0.648, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_8_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.1, -0.647, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_2_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.1, 0.647, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_10_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.816, -0.484, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_2_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.816, 0.484, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_10_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.632, -0.164, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_12_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.632, 0.164, L2_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_12_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.938, -0.164, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.938, 0.164, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.753, -0.483, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_4_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.753, 0.483, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_8_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.469, -0.648, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_4_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.469, 0.648, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_8_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.1, -0.647, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_2_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-4.1, 0.647, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_10_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.816, -0.484, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_2_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.816, 0.484, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_10_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.632, -0.164, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_12_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-3.632, 0.164, L3_SCORE_HEIGHT_METERS, CORAL_TO_L2_AND_L3_ROTATION.rotateBy(CORAL_12_OCLOCK_ROTATION)).relativeTo(FIELD_CENTER),
            new Pose3d(-5.065, -0.164, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-5.065, 0.164, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.817, -0.594, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.817, 0.594, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.532, -0.757, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.532, 0.757, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.036, -0.758, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-4.036, 0.758, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-3.752, -0.593, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-3.752, 0.593, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-3.504, -0.164, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER),
            new Pose3d(-3.504, 0.164, L4_SCORE_HEIGHT_METERS, CORAL_TO_VERTICAL_POSITION_ROTATION).relativeTo(FIELD_CENTER)
    ));
    public static final Pose3d PROCESSOR_LOCATION = new Pose3d(-2.786, -4.037, 0.431, new Rotation3d()).relativeTo(FIELD_CENTER);
    public static final Pose2d
            LEFT_FEEDER_POSITION = new Pose2d(FIELD_LENGTH_METERS, 0, new Rotation2d()),//TODO: AAAAAAAAA
            RIGHT_FEEDER_POSITION = new Pose2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS, new Rotation2d());

    public enum GamePieceType {
        CORAL(0.06, 0),
        ALGAE(0.15, 1);

        public final double originPointHeightOffGroundMeters;
        public final int id;

        GamePieceType(double originPointHeightOffGroundMeters, int id) {
            this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
            this.id = id;
        }

        public static String getNameByID(int id) {
            for (int i = 0; i < values().length; i++)
                if (values()[i].id == id)
                    return values()[i].toString();
            return "";
        }
    }
}
