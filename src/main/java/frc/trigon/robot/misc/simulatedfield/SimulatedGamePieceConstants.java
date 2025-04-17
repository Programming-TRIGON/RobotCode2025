package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.constants.FieldConstants;
import org.trigon.utilities.flippable.FlippablePose3d;
import org.trigon.utilities.flippable.FlippableTranslation2d;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePieceConstants {
    public static final double G_FORCE = 9.806;
    public static final double
            CORAL_INTAKE_TOLERANCE_METERS = 0.3,
            CORAL_FEEDER_INTAKE_TOLERANCE_METERS = 1,
            ALGAE_INTAKE_TOLERANCE_METERS = 0.3,
            CORAL_SCORING_TOLERANCE_METERS = 0.1,
            ALGAE_SCORING_TOLERANCE_METERS = 0.2;

    private static final double
            FIELD_WIDTH_METERS = FieldConstants.FIELD_WIDTH_METERS,
            FIELD_LENGTH_METERS = FieldConstants.FIELD_LENGTH_METERS;
    private static final Rotation3d CORAL_TO_VERTICAL_POSITION_ROTATION = new Rotation3d(0, Math.PI / 2, 0);

    /**
     * Stores all the game pieces.
     * Starts out with the game pieces the start on the field.
     */
    public static final ArrayList<SimulatedGamePiece>
            CORAL_ON_FIELD = new ArrayList<>(List.of(
            createNewCoral(new Pose3d(1.22, FIELD_WIDTH_METERS / 2, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            createNewCoral(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            createNewCoral(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            createNewCoral(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            createNewCoral(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            createNewCoral(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION))
    )),
            ALGAE_ON_FIELD = new ArrayList<>(List.of(
                    createNewAlgae(new Pose3d(1.22, FIELD_WIDTH_METERS / 2, 0.5, new Rotation3d())),
                    createNewAlgae(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.5, new Rotation3d())),
                    createNewAlgae(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.5, new Rotation3d())),
                    createNewAlgae(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2, 0.5, new Rotation3d())),
                    createNewAlgae(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.5, new Rotation3d())),
                    createNewAlgae(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.5, new Rotation3d()))
            ));

    private static final Translation3d
            REEF_CENTER_TO_L1_VECTOR = new Translation3d(0.652, 0.1643, 0.46),
            REEF_CENTER_TO_L2_VECTOR = new Translation3d(0.652, 0.1643, 0.6983),
            REEF_CENTER_TO_L3_VECTOR = new Translation3d(0.652, 0.1643, 1.1101),
            REEF_CENTER_TO_L4_VECTOR = new Translation3d(0.7796, 0.1643, 1.7345);
    private static final Rotation3d
            REEF_TO_2_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3),
            REEF_TO_4_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3 * 2),
            REEF_TO_6_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI),
            REEF_TO_8_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3 * 4),
            REEF_TO_10_OCLOCK_ROTATION = new Rotation3d(0, 0, Math.PI / 3 * 5),
            REEF_TO_12_OCLOCK_ROTATION = new Rotation3d(0, 0, 0);
    private static final Transform3d
            CORAL_TO_L1_ALIGNMENT = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, Units.degreesToRadians(15), 0)),
            CORAL_TO_L2_AND_L3_ALIGNMENT = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, -Units.degreesToRadians(35), 0)),
            CORAL_TO_L4_ALIGNMENT = new Transform3d(new Translation3d(0, 0, 0), CORAL_TO_VERTICAL_POSITION_ROTATION);
    private static final Transform3d
            RIGHT_BRANCH_TO_LEFT_BRANCH = new Transform3d(new Translation3d(0, -0.33, 0), new Rotation3d());
    public static final ArrayList<FlippablePose3d> CORAL_SCORING_LOCATIONS = calculatedCoralScoringLocations();
    public static final Pose3d PROCESSOR_LOCATION = new Pose3d(0, 0, 0, new Rotation3d());
    public static final FlippableTranslation2d
            LEFT_FEEDER_POSITION = new FlippableTranslation2d(0.923, 7.370, true),
            RIGHT_FEEDER_POSITION = new FlippableTranslation2d(0.923, 0.668, true);

    private static SimulatedGamePiece createNewCoral(Pose3d startingPose) {
        return new SimulatedGamePiece(startingPose, GamePieceType.CORAL);
    }

    private static SimulatedGamePiece createNewAlgae(Pose3d startingPose) {
        return new SimulatedGamePiece(startingPose, GamePieceType.ALGAE);
    }

    private static ArrayList<FlippablePose3d> calculatedCoralScoringLocations() {
        final ArrayList<FlippablePose3d> coralScoringPoses = new ArrayList<>();
        for (int level = 1; level <= 4; level++) {
            for (int clockFace = 2; clockFace <= 12; clockFace += 2) {
                coralScoringPoses.add(calculateCoralScorePose(level, clockFace, false));
                coralScoringPoses.add(calculateCoralScorePose(level, clockFace, true));
            }
        }
        return coralScoringPoses;
    }

    private static FlippablePose3d calculateCoralScorePose(int level, int clockFace, boolean isLeftBranch) {
        final Translation3d reefCenterToLevelVector;
        final Rotation3d reefToClockFaceRotation;
        final Transform3d coralAlignment;
        switch (level) {
            case 1 -> reefCenterToLevelVector = REEF_CENTER_TO_L1_VECTOR;
            case 2 -> reefCenterToLevelVector = REEF_CENTER_TO_L2_VECTOR;
            case 3 -> reefCenterToLevelVector = REEF_CENTER_TO_L3_VECTOR;
            case 4 -> reefCenterToLevelVector = REEF_CENTER_TO_L4_VECTOR;
            default -> reefCenterToLevelVector = new Translation3d();
        }
        switch (clockFace) {
            case 2 -> reefToClockFaceRotation = REEF_TO_2_OCLOCK_ROTATION;
            case 4 -> reefToClockFaceRotation = REEF_TO_4_OCLOCK_ROTATION;
            case 6 -> reefToClockFaceRotation = REEF_TO_6_OCLOCK_ROTATION;
            case 8 -> reefToClockFaceRotation = REEF_TO_8_OCLOCK_ROTATION;
            case 10 -> reefToClockFaceRotation = REEF_TO_10_OCLOCK_ROTATION;
            case 12 -> reefToClockFaceRotation = REEF_TO_12_OCLOCK_ROTATION;
            default -> reefToClockFaceRotation = new Rotation3d();
        }
        switch (level) {
            case 1 -> coralAlignment = CORAL_TO_L1_ALIGNMENT;
            case 2, 3 -> coralAlignment = CORAL_TO_L2_AND_L3_ALIGNMENT;
            case 4 -> coralAlignment = CORAL_TO_L4_ALIGNMENT;
            default -> coralAlignment = new Transform3d();
        }
        return new FlippablePose3d(new Pose3d(new Pose2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, new Rotation2d())).transformBy(new Transform3d(reefCenterToLevelVector.rotateBy(reefToClockFaceRotation), reefToClockFaceRotation)).transformBy(isLeftBranch ? RIGHT_BRANCH_TO_LEFT_BRANCH : new Transform3d()).transformBy(coralAlignment), true);
    }

    public enum GamePieceType {
        ALGAE(0.2, 0),
        CORAL(0.06, 1);

        public final double originPointHeightOffGroundMeters;
        public final int id;

        GamePieceType(double originPointHeightOffGroundMeters, int id) {
            this.originPointHeightOffGroundMeters = originPointHeightOffGroundMeters;
            this.id = id;
        }

        public static String getNameFromID(int id) {
            for (int i = 0; i < values().length; i++)
                if (values()[i].id == id)
                    return values()[i].toString();
            return "";
        }
    }
}
