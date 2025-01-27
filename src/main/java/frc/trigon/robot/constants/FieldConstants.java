package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.FilesHandler;

import java.io.IOException;
import java.util.HashMap;

public class FieldConstants {
    public static final double
            FIELD_WIDTH_METERS = FlippingUtil.fieldSizeY,
            FIELD_LENGTH_METERS = FlippingUtil.fieldSizeX;

    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = true;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    public static final int REEF_CLOCK_POSITIONS = 6;
    public static final Rotation2d CLOCK_POSITION_DIFFERENCE = Rotation2d.fromDegrees(Conversions.DEGREES_PER_ROTATIONS / REEF_CLOCK_POSITIONS);
    public static final Translation2d BLUE_REEF_CENTER_TRANSLATION = new Translation2d(4.48945, FIELD_WIDTH_METERS / 2);

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "2025-reefscape.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            tagIdToPose.put(aprilTag.ID, aprilTag.pose.transformBy(TAG_OFFSET));
        return tagIdToPose;
    }

    public enum ReefSide {
        LEFT(false),
        RIGHT(true);

        public final boolean doesFlipYTransformWhenFacingDriverStation;

        ReefSide(boolean doesFlipYTransformWhenFacingDriverStation) {
            this.doesFlipYTransformWhenFacingDriverStation = doesFlipYTransformWhenFacingDriverStation;
        }

        public boolean shouldFlipYTransform(ReefClockPosition reefClockPosition) {
            return doesFlipYTransformWhenFacingDriverStation ^ reefClockPosition.isFacingDriverStation; // In Java, ^ acts as an XOR (exclusive OR) operator, which fits in this case
        }
    }

    public enum ReefClockPosition {
        REEF_12_OCLOCK(false),
        REEF_2_OCLOCK(false),
        REEF_4_OCLOCK(true),
        REEF_6_OCLOCK(true),
        REEF_8_OCLOCK(true),
        REEF_10_OCLOCK(false);

        public final Rotation2d clockAngle;
        public final boolean isFacingDriverStation;
        public final int clockPosition;

        ReefClockPosition(boolean isFacingDriverStation) {
            this.clockAngle = calculateClockAngle();
            this.isFacingDriverStation = isFacingDriverStation;
            this.clockPosition = ordinal() == 0 ? 12 : ordinal() * 2;
        }

        private Rotation2d calculateClockAngle() {
            return CLOCK_POSITION_DIFFERENCE.times(-ordinal());
        }
    }
}