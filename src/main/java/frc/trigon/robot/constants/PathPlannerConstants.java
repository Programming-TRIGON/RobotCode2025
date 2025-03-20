package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.AutonomousCommands;
import frc.trigon.robot.commands.commandfactories.CoralCollectionCommands;
import frc.trigon.robot.commands.commandfactories.CoralPlacingCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.json.simple.parser.ParseException;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.LocalADStarAK;
import org.trigon.utilities.flippable.Flippable;

import java.io.IOException;

/**
 * A class that contains the constants and configurations for everything related to PathPlanner.
 */
public class PathPlannerConstants {
    public static final PathConstraints DRIVE_TO_REEF_CONSTRAINTS = new PathConstraints(2.5, 4, Units.degreesToRadians(450), Units.degreesToRadians(900));
    public static final double MINIMUM_DISTANCE_FROM_REEF_TO_OPEN_ELEVATOR = 2.2;
    public static final String DEFAULT_AUTO_NAME = "FourL4CoralsFromFeeder";
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final double FEEDFORWARD_SCALAR = 0.47;

    private static final PIDConstants
            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(0, 0, 0) :
            new PIDConstants(0, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(0, 0, 0) :
                    new PIDConstants(0, 0, 0);
    //            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
//            new PIDConstants(6, 0, 0) :
//            new PIDConstants(6, 0, 0.1),
//            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
//                    new PIDConstants(5, 0, 0) :
//                    new PIDConstants(8, 0, 0);
    private static final PPHolonomicDriveController AUTO_PATH_FOLLOWING_CONTROLLER = new PPHolonomicDriveController(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS
    );

    /**
     * Initializes PathPlanner. This needs to be called before any PathPlanner function can be used.
     */
    public static void init() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathfindingCommand.warmupCommand().schedule();
        configureAutoBuilder();
        registerCommands();
    }

    private static void configureAutoBuilder() {
        AutoBuilder.configure(
                RobotContainer.POSE_ESTIMATOR::getEstimatedRobotPose,
                RobotContainer.POSE_ESTIMATOR::resetPose,
                RobotContainer.SWERVE::getSelfRelativeVelocity,
                (chassisSpeeds -> RobotContainer.SWERVE.drivePathPlanner(chassisSpeeds, true)),
                AUTO_PATH_FOLLOWING_CONTROLLER,
                ROBOT_CONFIG,
                Flippable::isRedAlliance,
                RobotContainer.SWERVE
        );
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("PrepareL2", AutonomousCommands.getPrepareForScoringInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel.L2));
        NamedCommands.registerCommand("PrepareL4", AutonomousCommands.getPrepareForScoringInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel.L4));
        NamedCommands.registerCommand("RestElevator", ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST));
        NamedCommands.registerCommand("RestCoralIntake", CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.REST));
        NamedCommands.registerCommand("RestGripper", GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.REST));
        NamedCommands.registerCommand("CollectCoralFromFeeder", AutonomousCommands.getCollectCoralFromFeederCommand());
        NamedCommands.registerCommand("LoadCoral", CoralCollectionCommands.getLoadCoralCommand().until(RobotContainer.GRIPPER::hasGamePiece));
        NamedCommands.registerCommand("ScoreL2", AutonomousCommands.getScoreInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel.L2));
        NamedCommands.registerCommand("ScoreL4", AutonomousCommands.getScoreInReefFromGripperCommand(CoralPlacingCommands.ScoringLevel.L4));
    }
}