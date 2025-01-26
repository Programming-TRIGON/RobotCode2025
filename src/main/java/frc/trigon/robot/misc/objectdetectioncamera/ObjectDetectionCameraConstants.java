package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

public class ObjectDetectionCameraConstants {
    public static final int NUMBER_OF_GAME_PIECE_TYPES = SimulatedGamePieceConstants.GamePieceType.values().length;
    public static final Rotation3d BEST_ROTATION = new Rotation3d(0, Units.degreesToRadians(-38), 0);
}
