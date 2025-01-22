package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import org.littletonrobotics.junction.AutoLog;

public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class ObjectDetectionCameraInputs {
        /**
         * Whether there is at least one target or not for each game piece, by game piece index (type).
         */
        public boolean[] hasTarget = new boolean[SimulatedGamePieceConstants.GamePieceType.values().length];
        /**
         * Stores the yaws of all visible objects.
         * The first index is the game piece ID (type).
         * The second index is the index of the game piece's yaws, with the best yaw placed first (index 0).
         */
        public Rotation2d[][] visibleObjectYaws = new Rotation2d[SimulatedGamePieceConstants.GamePieceType.values().length][0];
    }
}
