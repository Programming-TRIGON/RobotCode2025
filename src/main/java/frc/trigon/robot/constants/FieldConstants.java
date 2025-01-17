package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.trigon.utilities.FilesHandler;
import org.trigon.utilities.flippable.FlippablePose2d;

import java.io.IOException;
import java.util.HashMap;

public class FieldConstants {
    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = true;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "2025-reefscape.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);//TODO: Change for year
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static final Transform2d REEF_SIDE_6_OCLOCK = new Transform2d(new Translation2d(0.3556, new Rotation2d()), new Rotation2d(Units.degreesToRadians(0)));
    private static final Transform2d REEF_SIDE_8_OCLOCK = new Transform2d(new Translation2d(0.3556, new Rotation2d()), new Rotation2d(Units.degreesToRadians(60)));
    private static final Transform2d REEF_SIDE_10_OCLOCK = new Transform2d(new Translation2d(0.3556, new Rotation2d()), new Rotation2d(Units.degreesToRadians(120)));
    private static final Transform2d REEF_SIDE_12_OCLOCK = new Transform2d(new Translation2d(0.3556, new Rotation2d()), new Rotation2d(Units.degreesToRadians(180)));
    private static final Transform2d REEF_SIDE_2_OCLOCK = new Transform2d(new Translation2d(0.3556, new Rotation2d()), new Rotation2d(Units.degreesToRadians(240)));
    private static final Transform2d REEF_SIDE_4_OCLOCK = new Transform2d(new Translation2d(0.3556, new Rotation2d()), new Rotation2d(Units.degreesToRadians(360)));

    private static final FlippablePose2d REEF_POSE = new FlippablePose2d(new Pose2d(4.48945, FlippingUtil.fieldSizeY / 2, new Rotation2d()), true);
    public static final FlippablePose2d REEF_6_OCLOCK_POSE = new FlippablePose2d(REEF_POSE.get().transformBy(REEF_SIDE_6_OCLOCK), true);
    public static final FlippablePose2d REEF_8_OCLOCK_POSE = new FlippablePose2d(REEF_POSE.get().transformBy(REEF_SIDE_8_OCLOCK), true);
    public static final FlippablePose2d REEF_10_OCLOCK_POSE = new FlippablePose2d(REEF_POSE.get().transformBy(REEF_SIDE_10_OCLOCK), true);
    public static final FlippablePose2d REEF_12_OCLOCK_POSE = new FlippablePose2d(REEF_POSE.get().transformBy(REEF_SIDE_12_OCLOCK), true);
    public static final FlippablePose2d REEF_2_OCLOCK_POSE = new FlippablePose2d(REEF_POSE.get().transformBy(REEF_SIDE_2_OCLOCK), true);
    public static final FlippablePose2d REEF_4_OCLOCK_POSE = new FlippablePose2d(REEF_POSE.get().transformBy(REEF_SIDE_4_OCLOCK), true);


    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            tagIdToPose.put(aprilTag.ID, aprilTag.pose.transformBy(TAG_OFFSET));
        return tagIdToPose;
    }
}
