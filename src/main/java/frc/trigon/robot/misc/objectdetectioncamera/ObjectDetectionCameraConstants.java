package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;

public class ObjectDetectionCameraConstants {
    public static final int NUMBER_OF_GAME_PIECE_TYPES = SimulatedGamePieceConstants.GamePieceType.values().length;
    public static final Rotation2d BEST_PITCH = Rotation2d.fromDegrees(-38);
}
