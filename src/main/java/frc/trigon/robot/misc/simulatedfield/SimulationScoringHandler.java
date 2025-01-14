package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;

public class SimulationScoringHandler {
    private static int POINTS = 0;

    private static void updateScoring(SimulatedGamePiece gamePiece) {
        if (gamePiece.isScored())
            return;
        final SimulatedGamePieceConstants.GamePieceType gamePieceType = gamePiece.getGamePieceType();

        if (gamePieceType.equals(SimulatedGamePieceConstants.GamePieceType.CORAL)) {
            for (int i = 0; i < SimulatedGamePieceConstants.NUMBER_OF_CORAL_BRANCHES; i++) {
                final Transform3d distanceFromScoreZone = SimulatedGamePieceConstants.L4_LOCATIONS[i].minus(gamePiece.getPose());
                if (distanceFromScoreZone.getTranslation().getNorm() < SimulatedGamePieceConstants.CORAL_SCORING_TOLERANCE_METERS) {
                    POINTS += SimulatedGamePieceConstants.L4_POINTS;
                    gamePiece.setScored(true);
                }
            }
        }

        if (gamePieceType.equals(SimulatedGamePieceConstants.GamePieceType.ALGAE)) {
            final Transform3d distanceFromScoreZone = SimulatedGamePieceConstants.PROCESSOR_LOCATION.minus(gamePiece.getPose());
            if (distanceFromScoreZone.getTranslation().getNorm() < SimulatedGamePieceConstants.ALGAE_SCORING_TOLERANCE_METERS) {
                POINTS += SimulatedGamePieceConstants.PROCESSOR_POINTS;
                gamePiece.setScored(true);
            }
        }
    }
}
