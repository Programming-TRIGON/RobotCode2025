package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePieceConstants {
    public static final double G_FORCE = 9.806;
    public static final double
            CORAL_INTAKE_TOLERANCE_METERS = 0.3,
            ALGAE_INTAKE_TOLERANCE_METERS = 0.4,
            CORAL_SCORING_TOLERANCE_METERS = 0.3,
            ALGAE_SCORING_TOLERANCE_METERS = 0.3;
    public static final int
            L4_TELEOP_POINTS = 5,
            L3_TELEOP_POINTS = 4,
            L2_TELEOP_POINTS = 3,
            L1_TELEOP_POINTS = 2,
            L4_AUTONOMOUS_POINTS = 7,
            L3_AUTONOMOUS_POINTS = 6,
            L2_AUTONOMOUS_POINTS = 4,
            L1_AUTONOMOUS_POINTS = 3,
            PROCESSOR_POINTS = 6;

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
            L4_SCORE_HEIGHT_METERS = 1.75;
    private static final Rotation3d
            CORAL_2_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3 * 2),
            CORAL_4_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3),
            CORAL_8_OCLOCK_ROTATION = new Rotation3d(0, 0, -Math.PI / 3),
            CORAL_10_OCLOCK_ROTATION = new Rotation3d(0, 0, -Math.PI / 3 * 2),
            CORAL_12_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI);
    private static final Pose3d FIELD_CENTER = new Pose3d(-FIELD_LENGTH_METERS / 2, -FIELD_WIDTH_METERS / 2, 0, new Rotation3d());

    public static final Pose3d[]
            L2_LOCATIONS = new Pose3d[]{
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
    },
            L3_LOCATIONS = new Pose3d[]{
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
            },
            L4_LOCATIONS = new Pose3d[]{
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
            };
    public static final Pose3d PROCESSOR_LOCATION = new Pose3d(-2.786, -4.037, 0.431, new Rotation3d()).relativeTo(FIELD_CENTER);

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
