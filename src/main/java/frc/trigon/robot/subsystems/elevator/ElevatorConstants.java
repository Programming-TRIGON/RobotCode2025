package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.ElevatorSimulation;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;

public class ElevatorConstants {
    private static final int
            MASTER_MOTOR_ID = 12,
            FOLLOWER_MOTOR_ID = 13;
    private static final String
            MASTER_MOTOR_NAME = "ElevatorMasterMotor",
            FOLLOWER_MOTOR_NAME = "ElevatorFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);

    private static final double GEAR_RATIO = 4.3333333333;
    private static final boolean SHOULD_FOLLOWER_OPPOSE_MASTER = true;
    static final double
            DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 80 : 20,
            DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 80 : 50;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double
            ELEVATOR_MASS_KILOGRAMS = 8,
            DRUM_RADIUS_METERS = 0.025,
            MINIMUM_ELEVATOR_HEIGHT_METERS = 0.73,
            MAXIMUM_ELEVATOR_HEIGHT_METERS = 1.8;
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            GEARBOX,
            GEAR_RATIO,
            ELEVATOR_MASS_KILOGRAMS,
            DRUM_RADIUS_METERS,
            MINIMUM_ELEVATOR_HEIGHT_METERS,//AFTER SEASON TODO: remove the need for this
            MAXIMUM_ELEVATOR_HEIGHT_METERS,
            SHOULD_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.25).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    public static final Pose3d FIRST_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(-0.120, 0, 0.131),
            new Rotation3d(0, 0, 0)
    );
    static final Pose3d SECOND_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(-0.120, 0, 0.1111),
            new Rotation3d(0, 0, 0)
    );
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            MAXIMUM_ELEVATOR_HEIGHT_METERS + 0.1,
            MINIMUM_ELEVATOR_HEIGHT_METERS,
            Color.kYellow
    );

    static final double FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS = 0.6;
    static final double DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    static final double POSITION_TOLERANCE_METERS = 0.04;
    static final double
            GRIPPER_HITTING_ELEVATOR_BASE_LOWER_BOUND_POSITION_ROTATIONS = 0.29,
            GRIPPER_HITTING_ELEVATOR_BASE_UPPER_BOUND_POSITION_ROTATIONS = 0.7,
            GRIPPER_HITTING_ALGAE_UPPER_BOUND_POSITION_ROTATIONS = 3;

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 40 : 6.5905;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.22774 : 0.2;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.066659 : 0.25513;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.74502 : 0.52756;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.30539 : 0.4;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.8;

        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR.getID(), SHOULD_FOLLOWER_OPPOSE_MASTER);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    public enum ElevatorState {
        REST(0, 0.7),
        SCORE_L1(0, 1),
        SCORE_L2(0, 1),
        SCORE_L3(0.4, 1),
        SCORE_L4(1.045, 1),
        COLLECT_ALGAE_FROM_L3(0.35, 1),
        REST_WITH_ALGAE(0, 0.3),
        SCORE_NET(1.045, 0.3);

        public final double targetPositionMeters;
        final double speedScalar;

        ElevatorState(double targetPositionMeters, double speedScalar) {
            this.targetPositionMeters = targetPositionMeters;
            this.speedScalar = speedScalar;
        }
    }
}