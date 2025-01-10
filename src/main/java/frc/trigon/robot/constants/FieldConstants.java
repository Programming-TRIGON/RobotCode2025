package frc.trigon.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class FieldConstants {
    public static final double
            FIELD_WIDTH_METERS = 8.05,
            FIELD_LENGTH_METERS = 17.55;
    public static final ArrayList<Translation2d> GAME_PIECES_STARTING_LOCATIONS = new ArrayList<>(List.of(
            new Translation2d(1.22, FIELD_WIDTH_METERS / 2),
            new Translation2d(1.22, FIELD_WIDTH_METERS / 2 - 1.83),
            new Translation2d(1.22, FIELD_WIDTH_METERS / 2 + 1.83),
            new Translation2d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2),
            new Translation2d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 - 1.83),
            new Translation2d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 + 1.83)
    ));


    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    AprilTagFieldLayout.loadFromResource("path/to/home/layout.json") :
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
}
