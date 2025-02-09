package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModule;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Gyro;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Signal;

public class SwerveConstants {
    static final Pigeon2Gyro GYRO = new Pigeon2Gyro(0, "SwerveGyro", RobotConstants.CANIVORE_NAME);
    public static final int
            FRONT_LEFT_ID = 1,
            FRONT_RIGHT_ID = 2;
    static final SwerveModule[] SWERVE_MODULES = new SwerveModule[]{
            new SwerveModule(1, -9.765625E-4, 0.04765354077913865 * 2),
            new SwerveModule(2, 0.25341796875, 0.04745360122422504 * 2),
            new SwerveModule(3, 0.20751953125, 0.05136841212501805 * 2),
            new SwerveModule(4, -0.05419921875, 0.04905587215351095 * 2)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(PathPlannerConstants.ROBOT_CONFIG.moduleLocations);
    static final double
            TRANSLATION_TOLERANCE_METERS = 0.05,
            ROTATION_TOLERANCE_DEGREES = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.3;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    static final double MAXIMUM_PID_ANGLE = 180;
    static final ProfiledPIDController PROFILED_ROTATION_PID_CONTROLLER = new ProfiledPIDController(
            RobotHardwareStats.isSimulation() ? 4 : 4.4,
            RobotHardwareStats.isSimulation() ? 0 : 0,
            RobotHardwareStats.isSimulation() ? 0 : 0,
            new TrapezoidProfile.Constraints(
                    RobotHardwareStats.isSimulation() ? 720 : 600,
                    RobotHardwareStats.isSimulation() ? 720 : 720
            )
    );
    static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(
            RobotHardwareStats.isSimulation() ? 5 : 5,
            RobotHardwareStats.isSimulation() ? 0 : 0,
            RobotHardwareStats.isSimulation() ? 0 : 0
    );
    public static final double
            MAXIMUM_SPEED_METERS_PER_SECOND = PathPlannerConstants.ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS,
            MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND = PathPlannerConstants.ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS / PathPlannerConstants.ROBOT_CONFIG.modulePivotDistance[0];

    static {
        configureGyro();
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.enableContinuousInput(-SwerveConstants.MAXIMUM_PID_ANGLE, SwerveConstants.MAXIMUM_PID_ANGLE);
    }

    private static void configureGyro() {
        final Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = -1.5493969917297363;
        config.MountPose.MountPosePitch = -0.31632718443870544;
        config.MountPose.MountPoseRoll = -0.9108231067657471;
        GYRO.applyConfiguration(config);
        GYRO.setSimulationYawVelocitySupplier(() -> RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond);

        GYRO.registerThreadedSignal(Pigeon2Signal.YAW, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }
}