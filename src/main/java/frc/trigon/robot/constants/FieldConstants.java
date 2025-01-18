package frc.trigon.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.FilesHandler;

import java.io.IOException;
import java.util.HashMap;

public class FieldConstants {
    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = true;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    public static final Rotation2d CLOCK_POSITION_DIFFERENCE = Rotation2d.fromDegrees(Conversions.DEGREES_PER_ROTATIONS / ReefClockPosition.values().length);
    public static final Translation2d REEF_CENTER_TRANSLATION = new Translation2d(4.5, 4); // TODO: Find actual middle pose

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "2025-reefscape.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);//TODO: Change for year
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
        LEFT(true),
        RIGHT(false);

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

        ReefClockPosition(boolean isFacingDriverStation) {
            this.clockAngle = CLOCK_POSITION_DIFFERENCE.times(-ordinal());
            this.isFacingDriverStation = isFacingDriverStation;
        }
    }
}
