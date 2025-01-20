package frc.trigon.robot.subsystems.coralintake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.misc.objectdetectioncamera.SimulationObjectDetectionCameraIO;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class CoralIntakeConstants {
    private static final int
            INTAKE_MOTOR_ID = 9,
            FUNNEL_MOTOR_ID = 10,
            ANGLE_MOTOR_ID = 11,
            ANGLE_ENCODER_ID = 11,
            BEAM_BREAK_PORT = 0;
    private static final String
            INTAKE_MOTOR_NAME = "CoralIntakeMotor",
            FUNNEL_MOTOR_NAME = "CoralFunnelMotor",
            ANGLE_MOTOR_NAME = "CoralAngleMotor",
            ANGLE_ENCODER_NAME = "CoralAngleEncoder",
            BEAM_BREAK_NAME = "CoralBeamBreak";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME),
            FUNNEL_MOTOR = new TalonFXMotor(FUNNEL_MOTOR_ID, FUNNEL_MOTOR_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME);
    static final SimpleSensor BEAM_BREAK = SimpleSensor.createDigitalSensor(BEAM_BREAK_PORT, BEAM_BREAK_NAME);

    private static final NeutralModeValue
            INTAKE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            FUNNEL_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            ANGLE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue
            INTAKE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FUNNEL_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            ANGLE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final double
            ANGLE_P = RobotHardwareStats.isSimulation() ? 300 : 0,
            ANGLE_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KA = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KG = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final double
            ANGLE_MOTION_MAGIC_CRUISE_VELOCITY = RobotHardwareStats.isSimulation() ? 3 : 0,
            ANGLE_MOTION_MAGIC_ACCELERATION = RobotHardwareStats.isSimulation() ? 3 : 0,
            ANGLE_MOTION_MAGIC_JERK = ANGLE_MOTION_MAGIC_ACCELERATION * 10;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseVelocitySign;
    private static final double
            INTAKE_MOTOR_GEAR_RATIO = 1,
            FUNNEL_MOTOR_GEAR_RATIO = 1,
            ANGLE_MOTOR_GEAR_RATIO = 100;
    private static final ForwardLimitSourceValue FORWARD_LIMIT_SOURCE_VALUE = ForwardLimitSourceValue.LimitSwitchPin;
    private static final ReverseLimitSourceValue REVERSE_LIMIT_SOURCE_VALUE = ReverseLimitSourceValue.LimitSwitchPin;
    private static final ForwardLimitTypeValue FORWARD_LIMIT_TYPE_VALUE = ForwardLimitTypeValue.NormallyOpen;
    private static final ReverseLimitTypeValue REVERSE_LIMIT_TYPE_VALUE = ReverseLimitTypeValue.NormallyOpen;
    private static final SensorDirectionValue ANGLE_ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final double
            ANGLE_ENCODER_MAGNET_OFFSET_VALUE = 0,
            ANGLE_ENCODER_DISCONTINUITY_POINT = 0.5;
    static final boolean FOC_ENABLED = true;

    private static final int
            INTAKE_MOTOR_AMOUNT = 1,
            FUNNEL_MOTOR_AMOUNT = 1,
            ANGLE_MOTOR_AMOUNT = 1;
    private static final DCMotor
            INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(INTAKE_MOTOR_AMOUNT),
            FUNNEL_GEARBOX = DCMotor.getKrakenX60Foc(FUNNEL_MOTOR_AMOUNT),
            ANGLE_GEARBOX = DCMotor.getKrakenX60Foc(ANGLE_MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            INTAKE_LENGTH_METERS = 0.3,
            INTAKE_MASS_KILOGRAMS = 5;
    private static final Rotation2d
            MAXIMUM_ANGLE = new Rotation2d(180),
            MINIMUM_ANGLE = new Rotation2d(0);//TODO: Find
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SimpleMotorSimulation
            INTAKE_SIMULATION = new SimpleMotorSimulation(
            INTAKE_GEARBOX,
            INTAKE_MOTOR_GEAR_RATIO,
            MOMENT_OF_INERTIA
    ),
            FUNNEL_SIMULATION = new SimpleMotorSimulation(
                    FUNNEL_GEARBOX,
                    FUNNEL_MOTOR_GEAR_RATIO,
                    MOMENT_OF_INERTIA
            );
    private static final SingleJointedArmSimulation
            ANGLE_MOTOR_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            INTAKE_LENGTH_METERS,
            INTAKE_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );
    private static final DoubleSupplier BEAM_BREAK_SIMULATION_VALUE_SUPPLIER = () -> SimulationObjectDetectionCameraIO.HAS_OBJECTS ? 1 : 0;

    static final SysIdRoutine.Config ANGLE_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(5).per(Units.Second),
            Units.Volts.of(9),
            Units.Second.of(1000)
    );

    static final Rotation2d INTAKE_ANGLE_FROM_GROUND = Rotation2d.fromDegrees(31.21);
    static final Pose3d INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(-0.1665, 0, 0.5651, new Rotation3d(0, INTAKE_ANGLE_FROM_GROUND.getRadians(), 0));
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d
            INTAKE_MECHANISM = new SpeedMechanism2d(
            "CoralIntakeMechanism",
            MAXIMUM_DISPLAYABLE_VELOCITY
    ),
            FUNNEL_MECHANISM = new SpeedMechanism2d(
                    "CoralFunnelMechanism",
                    MAXIMUM_DISPLAYABLE_VELOCITY
            );
    static final SingleJointedArmMechanism2d
            ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            "CoralAngleMechanism",
            INTAKE_LENGTH_METERS,
            Color.kRed
    );

    public static final double COLLECTION_LEDS_BLINKING_SPEED = 0.5;
    private static final double CORAL_COLLECTION_CONFIRMATION_TIME_THRESHOLD_SECONDS = 0.1;
    static final BooleanEvent CORAL_COLLECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            BEAM_BREAK::getBinaryValue
    ).debounce(CORAL_COLLECTION_CONFIRMATION_TIME_THRESHOLD_SECONDS);
    private static final double
            CORAL_DETECTION_CURRENT = 50,
            CORAL_DETECTION_TIME_THRESHOLD_SECONDS = 0.1;
    static final BooleanEvent EARLY_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> Math.abs(INTAKE_MOTOR.getSignal(TalonFXSignal.TORQUE_CURRENT)) > CORAL_DETECTION_CURRENT
    ).debounce(CORAL_DETECTION_TIME_THRESHOLD_SECONDS);
    public static final double
            COLLECTION_RUMBLE_DURATION_SECONDS = 0.1,
            COLLECTION_RUMBLE_POWER = 1;
    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1);

    static {
        configureIntakeMotor();
        configureFunnelMotor();
        configureAngleMotor();
        configureAngleEncoder();
        configureBeamBreak();
    }

    private static void configureIntakeMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = INTAKE_MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = INTAKE_MOTOR_INVERTED_VALUE;
        config.Feedback.RotorToSensorRatio = INTAKE_MOTOR_GEAR_RATIO;

        INTAKE_MOTOR.applyConfiguration(config);
        INTAKE_MOTOR.setPhysicsSimulation(INTAKE_SIMULATION);

        INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
    }

    private static void configureFunnelMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = FUNNEL_MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = FUNNEL_MOTOR_INVERTED_VALUE;
        config.Feedback.RotorToSensorRatio = FUNNEL_MOTOR_GEAR_RATIO;

        FUNNEL_MOTOR.applyConfiguration(config);
        FUNNEL_MOTOR.setPhysicsSimulation(FUNNEL_SIMULATION);

        FUNNEL_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        FUNNEL_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = ANGLE_MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = ANGLE_MOTOR_INVERTED_VALUE;
        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;
        config.Slot0.kG = ANGLE_KG;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

        config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ANGLE_MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ANGLE_MOTION_MAGIC_JERK;

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitType = FORWARD_LIMIT_TYPE_VALUE;
        config.HardwareLimitSwitch.ForwardLimitSource = FORWARD_LIMIT_SOURCE_VALUE;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitType = REVERSE_LIMIT_TYPE_VALUE;
        config.HardwareLimitSwitch.ReverseLimitSource = REVERSE_LIMIT_SOURCE_VALUE;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = ANGLE_ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_MAGNET_OFFSET_VALUE;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ANGLE_ENCODER_DISCONTINUITY_POINT;

        ANGLE_ENCODER.applyConfiguration(config);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void configureBeamBreak() {
        BEAM_BREAK.setSimulationSupplier(BEAM_BREAK_SIMULATION_VALUE_SUPPLIER);
    }

    public enum CoralIntakeState {
        LOAD(-12, -12, Rotation2d.fromDegrees(0)),
        PREPARE_FOR_LOADING(12, 12, Rotation2d.fromDegrees(0)),
        COLLECT(12, 12, Rotation2d.fromDegrees(180)),
        FEED(-12, -12, Rotation2d.fromDegrees(0)),
        EJECT(-12, -12, Rotation2d.fromDegrees(180)),
        REST(0, 0, Rotation2d.fromDegrees(0));

        public final double
                targetIntakeVoltage,
                targetFunnelVoltage;
        public final Rotation2d targetAngle;

        CoralIntakeState(double targetIntakeVoltage, double targetFunnelVoltage, Rotation2d targetAngle) {
            this.targetIntakeVoltage = targetIntakeVoltage;
            this.targetFunnelVoltage = targetFunnelVoltage;
            this.targetAngle = targetAngle;
        }
    }
}