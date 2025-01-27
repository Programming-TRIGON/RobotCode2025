package frc.trigon.robot.misc.simulatedfield;

import edu.wpi.first.math.geometry.Pose3d;
import org.trigon.utilities.flippable.FlippablePose3d;

import java.util.ArrayList;

public class SimulationScoringHandler {
    public static void checkGamePieceScored(SimulatedGamePiece gamePiece) {
        if (gamePiece.gamePieceType.equals(SimulatedGamePieceConstants.GamePieceType.CORAL))
            checkCoralScored(gamePiece);
        if (gamePiece.gamePieceType.equals(SimulatedGamePieceConstants.GamePieceType.ALGAE))
            checkAlgaeScored(gamePiece);
    }

    private static void checkCoralScored(SimulatedGamePiece coral) {
        final ArrayList<FlippablePose3d> scoreLocations = SimulatedGamePieceConstants.CORAL_SCORING_LOCATIONS;
        for (FlippablePose3d scoreLocation : scoreLocations) {
            final Pose3d flippedPose = scoreLocation.get();
            if (isGamePieceScored(coral, flippedPose, SimulatedGamePieceConstants.CORAL_SCORING_TOLERANCE_METERS)) {
                coral.isScored = true;
                coral.updatePose(flippedPose);
                scoreLocations.remove(scoreLocation);
                return;
            }
        }
    }

    private static void checkAlgaeScored(SimulatedGamePiece algae) {
        if (!isGamePieceScored(algae, SimulatedGamePieceConstants.PROCESSOR_LOCATION, SimulatedGamePieceConstants.ALGAE_SCORING_TOLERANCE_METERS))
            return;

        algae.isScored = true;
        SimulationFieldHandler.getSimulatedAlgae().remove(algae);
    }

    private static boolean isGamePieceScored(SimulatedGamePiece gamePiece, Pose3d scoreLocation, double scoringToleranceMeters) {
        final double distanceFromScoreZoneMeters = gamePiece.getDistanceFromPoseMeters(scoreLocation);
        return distanceFromScoreZoneMeters < scoringToleranceMeters;
    }
}
