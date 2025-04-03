package frc.trigon.robot.subsystems.coralintake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
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
            BEAM_BREAK_CHANNEL = 1,
            DISTANCE_SENSOR_CHANNEL = 0;
    private static final String
            INTAKE_MOTOR_NAME = "CoralIntakeMotor",
            FUNNEL_MOTOR_NAME = "CoralFunnelMotor",
            ANGLE_MOTOR_NAME = "CoralAngleMotor",
            ANGLE_ENCODER_NAME = "CoralAngleEncoder",
            BEAM_BREAK_NAME = "CoralBeamBreak",
            DISTANCE_SENSOR_NAME = "CoralDistanceSensor";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME),
            FUNNEL_MOTOR = new TalonFXMotor(FUNNEL_MOTOR_ID, FUNNEL_MOTOR_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME);
    static final SimpleSensor
            BEAM_BREAK = SimpleSensor.createDigitalSensor(BEAM_BREAK_CHANNEL, BEAM_BREAK_NAME),
            BACKUP_BEAM_BREAK = SimpleSensor.createDigitalSensor(3, "CoralBackupBeamBreak"),
            DISTANCE_SENSOR = SimpleSensor.createDutyCycleSensor(DISTANCE_SENSOR_CHANNEL, DISTANCE_SENSOR_NAME);

    private static final double
            INTAKE_MOTOR_GEAR_RATIO = 1.3,
            FUNNEL_MOTOR_GEAR_RATIO = 4;
    static final double ANGLE_MOTOR_GEAR_RATIO = 39.577227;
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET_VALUE = -0.024915 - 0.0067957 - 0.013338;
    static final double ANGLE_ENCODER_POSITION_OFFSET_VALUE = RobotHardwareStats.isSimulation() ? 0 : -0.053944 - ANGLE_ENCODER_GRAVITY_OFFSET_VALUE;//0.213
    private static final double
            DISTANCE_SENSOR_SCALING_SLOPE = 0.0002,
            DISTANCE_SENSOR_SCALING_INTERCEPT_POINT = -200;
    static final boolean FOC_ENABLED = true;

    private static final int
            INTAKE_MOTOR_AMOUNT = 1,
            FUNNEL_MOTOR_AMOUNT = 1,
            ANGLE_MOTOR_AMOUNT = 1;
    private static final DCMotor
            INTAKE_GEARBOX = DCMotor.getFalcon500(INTAKE_MOTOR_AMOUNT),
            FUNNEL_GEARBOX = DCMotor.getFalcon500(FUNNEL_MOTOR_AMOUNT),
            ANGLE_GEARBOX = DCMotor.getKrakenX60(ANGLE_MOTOR_AMOUNT);

    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            INTAKE_LENGTH_METERS = 0.44,
            INTAKE_MASS_KILOGRAMS = 8;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-48),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(150);
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
    private static final DoubleSupplier
            BEAM_BREAK_SIMULATION_VALUE_SUPPLIER = () -> SimulationFieldHandler.isHoldingCoral() && !SimulationFieldHandler.isCoralInGripper() ? 1 : 0,
            DISTANCE_SENSOR_SIMULATION_VALUE_SUPPLIER = () -> SimulationFieldHandler.isHoldingCoral() ? 0 : 1000;

    static final SysIdRoutine.Config ANGLE_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    static final Pose3d INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0.344, 0, 0.3291),
            new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(137), 0)
    );
    static final Transform3d
            CORAL_INTAKE_ORIGIN_POINT_TO_CORAL_COLLECTION_TRANSFORM = new Transform3d(
            new Translation3d(0.44, 0, 0),
            new Rotation3d(0, 0, 0)
    ),
            CORAL_INTAKE_ORIGIN_POINT_TO_CORAL_VISUALIZATION_TRANSFORM = new Transform3d(
                    new Translation3d(0.33, 0, 0.015),
                    new Rotation3d(0, 0, 0)
            );
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

    public static final double
            COLLECTION_RUMBLE_DURATION_SECONDS = 0.7,
            COLLECTION_RUMBLE_POWER = 1;
    private static final double
            CORAL_COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS = 0.1,
            EARLY_CORAL_COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS = 0.06;
    private static final double EARLY_COLLECTION_DETECTION_DISTANCE_CENTIMETRES = 15;
    static final BooleanEvent EARLY_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> DISTANCE_SENSOR.getScaledValue() < EARLY_COLLECTION_DETECTION_DISTANCE_CENTIMETRES
    ).debounce(EARLY_CORAL_COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS);
    static final BooleanEvent CORAL_COLLECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> BEAM_BREAK.getBinaryValue() && BACKUP_BEAM_BREAK.getBinaryValue()
    ).debounce(CORAL_COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS);
    static final BooleanEvent OVERRIDE_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            EARLY_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT
    ).debounce(0.6);

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.5);
    static final double
            PULSING_ON_PERIOD_SECONDS = 0.22,
            PULSING_OFF_PERIOD_SECONDS = 0.1,
            PULSING_INTAKE_MOTOR_VOLTAGE = 8,
            PULSING_FUNNEL_MOTOR_VOLTAGE = CoralIntakeState.COLLECT_FROM_FLOOR.targetFunnelVoltage,
            PULSING_ANGLE_DEGREES = 90;

    static {
        configureIntakeMotor();
        configureFunnelMotor();
        configureAngleMotor();
        configureAngleEncoder();
        configureBeamBreak();
        configureDistanceSensor();
    }

    private static void configureIntakeMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.RotorToSensorRatio = INTAKE_MOTOR_GEAR_RATIO;

        INTAKE_MOTOR.applyConfiguration(config);
        INTAKE_MOTOR.setPhysicsSimulation(INTAKE_SIMULATION);

        INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        INTAKE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFunnelMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ANGLE_ENCODER.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 75 : 30;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 1.6663 : 1;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.074947 : 0.26083;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 8.7544 : 4.3;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0.04;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.27712 : 0.50326;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = RobotHardwareStats.isSimulation() ? StaticFeedforwardSignValue.UseClosedLoopSign : StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 12 / config.Slot0.kV : 2;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 6 : 8;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.128662 - ANGLE_ENCODER_POSITION_OFFSET_VALUE;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotation2d.fromDegrees(145).minus(Rotation2d.fromRotations(ANGLE_ENCODER_POSITION_OFFSET_VALUE)).getRotations();

        config.CurrentLimits.SupplyCurrentLimit = 60;

        config.Feedback.VelocityFilterTimeConstant = 0;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET_VALUE;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ANGLE_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void configureBeamBreak() {
        BEAM_BREAK.setSimulationSupplier(BEAM_BREAK_SIMULATION_VALUE_SUPPLIER);
    }

    private static void configureDistanceSensor() {
        DISTANCE_SENSOR.setScalingConstants(DISTANCE_SENSOR_SCALING_SLOPE, DISTANCE_SENSOR_SCALING_INTERCEPT_POINT);
        DISTANCE_SENSOR.setSimulationSupplier(DISTANCE_SENSOR_SIMULATION_VALUE_SUPPLIER);
    }

    public enum CoralIntakeState {
        LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK(-4.5, -4, Rotation2d.fromDegrees(143)),
        LOAD_CORAL_TO_GRIPPER_NOT_SEEING_GAME_PIECE_WITH_BEAM_BREAK(-4.5, 0, LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK.targetAngle),
        UNLOAD_CORAL_FROM_GRIPPER(6, 2, Rotation2d.fromDegrees(141)),
        CENTER_CORAL(8, 2, LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK.targetAngle),
        COLLECT_FROM_FLOOR(7, 5, Rotation2d.fromDegrees(-46)),
        COLLECT_FROM_FEEDER(6, COLLECT_FROM_FLOOR.targetFunnelVoltage, Rotation2d.fromDegrees(90)),
        EJECT(-3, -1, Rotation2d.fromDegrees(45)),
        REST(0, 0, LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK.targetAngle),
        SCORE_L1_BOOST(-3, -3, Rotation2d.fromDegrees(33)),
        SCORE_L1(-2, 3, Rotation2d.fromDegrees(33)),
        COLLECT_ALGAE_FROM_FLOOR(5, 0, Rotation2d.fromDegrees(-15)),
        PREPARE_SCORE_ALGAE_IN_PROCESSOR(5, 0, Rotation2d.fromDegrees(15)),
        SCORE_ALGAE_IN_PROCESSOR(-2, 0, PREPARE_SCORE_ALGAE_IN_PROCESSOR.targetAngle);

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