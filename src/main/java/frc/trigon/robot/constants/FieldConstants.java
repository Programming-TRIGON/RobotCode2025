package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.FilesHandler;
import org.trigon.utilities.flippable.FlippablePose2d;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

public class FieldConstants {
    public static final double
            FIELD_WIDTH_METERS = FlippingUtil.fieldSizeY,
            FIELD_LENGTH_METERS = FlippingUtil.fieldSizeX;
    private static final List<Integer> I_HATE_YOU = List.of(
            13, 12, 16, 15, 14, 4, 5, 3, 2, 1
    );

    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    public static final FlippablePose2d AUTO_FIND_CORAL_POSE_LEFT = new FlippablePose2d(3.3, 5.5, Rotation2d.fromDegrees(165), true);
    public static final FlippablePose2d AUTO_FIND_CORAL_POSE_RIGHT = new FlippablePose2d(3.3, FieldConstants.FIELD_WIDTH_METERS - 5.5, Rotation2d.fromDegrees(-165), true);

    public static final int REEF_CLOCK_POSITIONS = 6;
    public static final Rotation2d REEF_CLOCK_POSITION_DIFFERENCE = Rotation2d.fromDegrees(Conversions.DEGREES_PER_ROTATIONS / REEF_CLOCK_POSITIONS);
    public static final Rotation2d[] REEF_CLOCK_ANGLES = ReefClockPosition.getClockAngles();
    public static final Translation2d BLUE_REEF_CENTER_TRANSLATION = new Translation2d(4.48945, FIELD_WIDTH_METERS / 2);
    public static final double
            REEF_CENTER_TO_TARGET_SCORING_POSITION_X_TRANSFORM_METERS = 1.3,
            REEF_CENTER_TO_TARGET_SCORING_POSITION_Y_TRANSFORM_METERS = 0.17,
            REEF_CENTER_TO_TARGET_ALGAE_COLLECTION_POSITION_X_TRANSFORM_METERS = 1.6;

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "field_calibration.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags()) {
            if (!I_HATE_YOU.contains(aprilTag.ID))
                tagIdToPose.put(aprilTag.ID, aprilTag.pose.transformBy(TAG_OFFSET));
        }

        return tagIdToPose;
    }

    public enum ReefSide {
        RIGHT(true),
        LEFT(false);

        public final boolean doesFlipYTransformWhenFacingDriverStation;

        ReefSide(boolean doesFlipYTransformWhenFacingDriverStation) {
            this.doesFlipYTransformWhenFacingDriverStation = doesFlipYTransformWhenFacingDriverStation;
        }

        public boolean shouldFlipYTransform(ReefClockPosition reefClockPosition) {
            return doesFlipYTransformWhenFacingDriverStation ^ reefClockPosition.isFacingDriverStation; // In Java, ^ acts as an XOR (exclusive OR) operator, which fits in this case
        }
    }

    public enum ReefClockPosition {
        REEF_12_OCLOCK(true),
        REEF_2_OCLOCK(true),
        REEF_4_OCLOCK(true),
        REEF_6_OCLOCK(true),
        REEF_8_OCLOCK(true),
        REEF_10_OCLOCK(true);

        public final Rotation2d clockAngle;
        public final boolean isFacingDriverStation;
        public final int clockPosition;

        ReefClockPosition(boolean isFacingDriverStation) {
            this.clockAngle = calculateClockAngle();
            this.isFacingDriverStation = isFacingDriverStation;
            this.clockPosition = ordinal() == 0 ? 12 : ordinal() * 2;
        }

        public static Rotation2d[] getClockAngles() {
            final Rotation2d[] clockAngles = new Rotation2d[ReefClockPosition.values().length];
            for (int i = 0; i < clockAngles.length; i++)
                clockAngles[i] = ReefClockPosition.values()[i].clockAngle;

            return clockAngles;
        }

        private Rotation2d calculateClockAngle() {
            return REEF_CLOCK_POSITION_DIFFERENCE.times(-ordinal());
        }
    }
}
