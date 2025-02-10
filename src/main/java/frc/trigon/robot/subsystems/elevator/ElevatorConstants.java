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

    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final double GEAR_RATIO = 7.222222;
    private static final boolean FOLLOWER_MOTOR_OPPOSES_MASTER = true;
    private static final double
            P = RobotHardwareStats.isSimulation() ? 40 : 0.6,
            I = RobotHardwareStats.isSimulation() ? 0 : 0,
            D = RobotHardwareStats.isSimulation() ? 0.22774 : 0,
            KS = RobotHardwareStats.isSimulation() ? 0.066659 : 0.079427,
            KV = RobotHardwareStats.isSimulation() ? 0.74502 : 0.88,
            KG = RobotHardwareStats.isSimulation() ? 0.30539 : 0.42,
            KA = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final double
            MOTION_MAGIC_CRUISE_VELOCITY = RobotHardwareStats.isSimulation() ? 80 : 10,
            MOTION_MAGIC_ACCELERATION = RobotHardwareStats.isSimulation() ? 80 : 50,
            MOTION_MAGIC_JERK = MOTION_MAGIC_ACCELERATION * 10;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseClosedLoopSign;
    private static final ReverseLimitSourceValue REVERSE_LIMIT_SOURCE_VALUE = ReverseLimitSourceValue.LimitSwitchPin;
    private static final ForwardLimitSourceValue FORWARD_LIMIT_SOURCE_VALUE = ForwardLimitSourceValue.LimitSwitchPin;
    private static final ReverseLimitTypeValue REVERSE_LIMIT_TYPE_VALUE = ReverseLimitTypeValue.NormallyOpen;
    private static final ForwardLimitTypeValue FORWARD_LIMIT_TYPE_VALUE = ForwardLimitTypeValue.NormallyOpen;
    private static final double REVERSE_LIMIT_SWITCH_RESET_POSITION = 0;
    private static final double
            REVERSE_SOFT_LIMIT_THRESHOLD_ROTATIONS = 0,
            FORWARD_SOFT_LIMIT_THRESHOLD_ROTATIONS = 6.6;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double
            ELEVATOR_MASS_KILOGRAMS = 8,
            DRUM_RADIUS_METERS = 0.025,
            RETRACTED_ELEVATOR_HEIGHT_METERS = 0.73, // AFTER SEASON TODO: remove the need for this
            MAXIMUM_ELEVATOR_HEIGHT_METERS = 1.8;
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            GEARBOX,
            GEAR_RATIO,
            ELEVATOR_MASS_KILOGRAMS,
            DRUM_RADIUS_METERS,
            RETRACTED_ELEVATOR_HEIGHT_METERS,
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
    static final double MECHANISM_EXTRA_VISUALIZATION_LENGTH_METERS = 0.1;
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            MAXIMUM_ELEVATOR_HEIGHT_METERS + MECHANISM_EXTRA_VISUALIZATION_LENGTH_METERS,
            RETRACTED_ELEVATOR_HEIGHT_METERS,
            Color.kYellow
    );

    static final double FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS = 0.6;
    static final double DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    static final double POSITION_TOLERANCE_METERS = 0.02;
    static final double
            GRIPPER_HITTING_ELEVATOR_BASE_LOWER_BOUND_POSITION_ROTATIONS = 0.29,
            GRIPPER_HITTING_ELEVATOR_BASE_UPPER_BOUND_POSITION_ROTATIONS = 0.7;

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

//        config.HardwareLimitSwitch.ReverseLimitEnable = true;
//        config.HardwareLimitSwitch.ReverseLimitSource = REVERSE_LIMIT_SOURCE_VALUE;
//        config.HardwareLimitSwitch.ReverseLimitType = REVERSE_LIMIT_TYPE_VALUE;
//        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
//        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = REVERSE_LIMIT_SWITCH_RESET_POSITION;

//        config.HardwareLimitSwitch.ForwardLimitEnable = true;
//        config.HardwareLimitSwitch.ForwardLimitSource = FORWARD_LIMIT_SOURCE_VALUE;
//        config.HardwareLimitSwitch.ForwardLimitType = FORWARD_LIMIT_TYPE_VALUE;
//        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT_THRESHOLD_ROTATIONS;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT_THRESHOLD_ROTATIONS;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);

        MASTER_MOTOR.setPosition(0);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSES_MASTER);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    public enum ElevatorState {
        REST(0),
        SCORE_L1(0),
        SCORE_L2(0),
        SCORE_L3(0.43),
        SCORE_L4(1.02),
        COLLECT_ALGAE_FROM_L3(0.365),
        SCORE_NET(1.04);

        public final double targetPositionMeters;

        ElevatorState(double targetPositionMeters) {
            this.targetPositionMeters = targetPositionMeters;
        }
    }
}