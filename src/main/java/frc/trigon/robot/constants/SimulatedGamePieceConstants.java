package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.trigon.robot.misc.simulatedfield.SimulatedAlgae;
import frc.trigon.robot.misc.simulatedfield.SimulatedCoral;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePiece;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePieceConstants {
    private static final double
            FIELD_WIDTH_METERS = FieldConstants.FIELD_WIDTH_METERS,
            FIELD_LENGTH_METERS = FieldConstants.FIELD_LENGTH_METERS;
    private static final Rotation3d CORAL_TO_VERTICAL_POSITION_ROTATION = new Rotation3d(0, Math.PI / 2, 0);

    public static final ArrayList<SimulatedGamePiece>
            CORAL_ON_FIELD = new ArrayList<>(List.of(
            new SimulatedCoral(new Pose3d(1.22, FIELD_WIDTH_METERS / 2, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            new SimulatedCoral(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            new SimulatedCoral(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            new SimulatedCoral(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            new SimulatedCoral(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION)),
            new SimulatedCoral(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.15, CORAL_TO_VERTICAL_POSITION_ROTATION))
    )),
            ALGAE_ON_FIELD = new ArrayList<>(List.of(
                    new SimulatedAlgae(new Pose3d(1.22, FIELD_WIDTH_METERS / 2, 0.5, new Rotation3d())),
                    new SimulatedAlgae(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.5, new Rotation3d())),
                    new SimulatedAlgae(new Pose3d(1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.5, new Rotation3d())),
                    new SimulatedAlgae(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2, 0.5, new Rotation3d())),
                    new SimulatedAlgae(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 - 1.83, 0.5, new Rotation3d())),
                    new SimulatedAlgae(new Pose3d(FIELD_LENGTH_METERS - 1.22, FIELD_WIDTH_METERS / 2 + 1.83, 0.5, new Rotation3d()))
            ));
}
