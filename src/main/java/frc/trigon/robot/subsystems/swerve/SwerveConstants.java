package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.swervemodule.SwerveModule;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Gyro;
import org.trigon.hardware.phoenix6.pigeon2.Pigeon2Signal;

public class SwerveConstants {
    private static final int GYRO_ID = 0;
    private static final String GYRO_NAME = "SwerveGyro";
    static final Pigeon2Gyro GYRO = new Pigeon2Gyro(GYRO_ID, GYRO_NAME, RobotConstants.CANIVORE_NAME);

    public static final int
            FRONT_LEFT_ID = 1,
            FRONT_RIGHT_ID = 2;
    private static final int
            REAR_LEFT_ID = 3,
            REAR_RIGHT_ID = 4;
    private static final double
            FRONT_LEFT_STEER_ENCODER_OFFSET = -0.05859375 + 0.5,
            FRONT_RIGHT_STEER_ENCODER_OFFSET = 0.210693359375 - 0.5,
            REAR_LEFT_STEER_ENCODER_OFFSET = 0.3525390625 - 0.5,
            REAR_RIGHT_STEER_ENCODER_OFFSET = 0.385498046875 - 0.5;
    private static final double
            FRONT_LEFT_WHEEL_DIAMETER = 0.046820512231028774 * 2,
            FRONT_RIGHT_WHEEL_DIAMETER = 0.04970578534013391 * 2,
            REAR_LEFT_WHEEL_DIAMETER = 0.04439747411839345 * 2,
            REAR_RIGHT_WHEEL_DIAMETER = 0.04580481274752353 * 2;
    static final SwerveModule[] SWERVE_MODULES = new SwerveModule[]{
            new SwerveModule(FRONT_LEFT_ID, FRONT_LEFT_STEER_ENCODER_OFFSET, FRONT_LEFT_WHEEL_DIAMETER),
            new SwerveModule(FRONT_RIGHT_ID, FRONT_RIGHT_STEER_ENCODER_OFFSET, FRONT_RIGHT_WHEEL_DIAMETER),
            new SwerveModule(REAR_LEFT_ID, REAR_LEFT_STEER_ENCODER_OFFSET, REAR_LEFT_WHEEL_DIAMETER),
            new SwerveModule(REAR_RIGHT_ID, REAR_RIGHT_STEER_ENCODER_OFFSET, REAR_RIGHT_WHEEL_DIAMETER)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(PathPlannerConstants.ROBOT_CONFIG.moduleLocations);
    static final double
            TRANSLATION_TOLERANCE_METERS = 0.02,
            ROTATION_TOLERANCE_DEGREES = 1.5,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.3;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    public static final double
            MAXIMUM_SPEED_METERS_PER_SECOND = PathPlannerConstants.ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS,
            MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND = PathPlannerConstants.ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS / PathPlannerConstants.ROBOT_CONFIG.modulePivotDistance[0];

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(5, 0, 0) :
//            new PIDConstants(5, 0, 0.24),
            new PIDConstants(4.5, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(4, 0, 0) :
                    new PIDConstants(8, 0, 0.1);
    private static final double
            MAXIMUM_ROTATION_VELOCITY = RobotHardwareStats.isSimulation() ? 720 : Units.radiansToDegrees(MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND),
            MAXIMUM_ROTATION_ACCELERATION = RobotHardwareStats.isSimulation() ? 720 : 1200;
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAXIMUM_ROTATION_VELOCITY,
            MAXIMUM_ROTATION_ACCELERATION
    );
    static final double MAXIMUM_PID_ANGLE = 180;
    static final ProfiledPIDController PROFILED_ROTATION_PID_CONTROLLER = new ProfiledPIDController(
            PROFILED_ROTATION_PID_CONSTANTS.kP,
            PROFILED_ROTATION_PID_CONSTANTS.kI,
            PROFILED_ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    static final PIDController
            X_TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    ),
            Y_TRANSLATION_PID_CONTROLLER = new PIDController(
                    TRANSLATION_PID_CONSTANTS.kP,
                    TRANSLATION_PID_CONSTANTS.kI,
                    TRANSLATION_PID_CONSTANTS.kD
            );

    static {
        configureGyro();
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.enableContinuousInput(-SwerveConstants.MAXIMUM_PID_ANGLE, SwerveConstants.MAXIMUM_PID_ANGLE);
    }

    private static void configureGyro() {
        final Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = -90.09468078613281;
        config.MountPose.MountPosePitch = -1.1334741115570068;
        config.MountPose.MountPoseRoll = -1.7601540088653564;
        GYRO.applyConfiguration(config);
        GYRO.setSimulationYawVelocitySupplier(() -> RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond);

        GYRO.registerThreadedSignal(Pigeon2Signal.YAW, PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }
}