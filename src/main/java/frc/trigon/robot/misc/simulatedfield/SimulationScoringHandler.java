package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;
import org.littletonrobotics.junction.Logger;

public class SimulationScoringHandler {
    private static int SCORE = 0;

    public static void update() {
        logScore();
    }

    public static void checkGamePieceScored(SimulatedGamePiece gamePiece) {
        if (gamePiece.gamePieceType.equals(SimulatedGamePieceConstants.GamePieceType.CORAL))
            checkCoralScored(gamePiece);
        else if (gamePiece.gamePieceType.equals(SimulatedGamePieceConstants.GamePieceType.ALGAE))
            checkAlgaeScored(gamePiece);
    }

    private static void logScore() {
        Logger.recordOutput("SimulatedGameScore", SCORE);
    }

    private static void checkCoralScored(SimulatedGamePiece coral) {
        for (int i = 1; i <= 4; i++)
            if (isCoralScoredOnLevel(coral, i))
                return;
    }

    private static void checkAlgaeScored(SimulatedGamePiece algae) {
        if (!isGamePieceScored(algae, SimulatedGamePieceConstants.PROCESSOR_LOCATION, SimulatedGamePieceConstants.ALGAE_SCORING_TOLERANCE_METERS))
            return;

        SCORE += SimulatedGamePieceConstants.PROCESSOR_POINTS;
        algae.isScored = true;
    }

    private static boolean isCoralScoredOnLevel(SimulatedGamePiece coral, int level) {
        final Pose3d[] scoreLocations = getCoralScoreLocationsFromLevel(level);
        for (Pose3d scoreLocation : scoreLocations) {
            if (isGamePieceScored(coral, scoreLocation, SimulatedGamePieceConstants.CORAL_SCORING_TOLERANCE_METERS)) {
                incrementScoreFromCoralLevel(level);
                coral.isScored = true;
                return true;
            }
        }
        return false;
    }

    private static boolean isGamePieceScored(SimulatedGamePiece gamePiece, Pose3d scoreLocation, double scoringToleranceMeters) {
        final double distanceFromScoreZoneMeters = gamePiece.getDistanceMeters(scoreLocation);
        return distanceFromScoreZoneMeters < scoringToleranceMeters;
    }

    private static Pose3d[] getCoralScoreLocationsFromLevel(int level) {
        return switch (level) {
//            case 1 -> SimulatedGamePieceConstants.L1_LOCATIONS;
            case 2 -> SimulatedGamePieceConstants.L2_LOCATIONS;
            case 3 -> SimulatedGamePieceConstants.L3_LOCATIONS;
            case 4 -> SimulatedGamePieceConstants.L4_LOCATIONS;
            default -> new Pose3d[0];
        };
    }

    private static void incrementScoreFromCoralLevel(int level) {
        if (level == 1)
            SCORE += SimulatedGamePieceConstants.L1_POINTS;
        if (level == 2)
            SCORE += SimulatedGamePieceConstants.L2_POINTS;
        if (level == 3)
            SCORE += SimulatedGamePieceConstants.L3_POINTS;
        if (level == 4)
            SCORE += SimulatedGamePieceConstants.L4_POINTS;
    }
}
