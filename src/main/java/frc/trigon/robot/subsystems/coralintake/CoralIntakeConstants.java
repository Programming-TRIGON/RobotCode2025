package frc.trigon.robot.subsystems.coralintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.ElevatorSimulation;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class CoralIntakeConstants {
    private static final int
            INTAKE_MOTOR_ID = 9,
            FUNNEL_MOTOR_ID = 10,
            MASTER_ELEVATOR_MOTOR_ID = 11,
            FOLLOWER_ELEVATOR_MOTOR_ID = 12,
            BEAM_BREAK_PORT = 0;
    private static final String
            INTAKE_MOTOR_NAME = "CoralIntakeMotor",
            FUNNEL_MOTOR_NAME = "CoralFunnelMotor",
            MASTER_ANGLE_MOTOR_NAME = "CoralMasterElevatorMotor",
            FOLLOWER_ANGLE_MOTOR_NAME = "CoralFollowerElevatorMotor",
            BEAM_BREAK_NAME = "CoralBeamBreak";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME),
            FUNNEL_MOTOR = new TalonFXMotor(FUNNEL_MOTOR_ID, FUNNEL_MOTOR_NAME),
            MASTER_ELEVATOR_MOTOR = new TalonFXMotor(MASTER_ELEVATOR_MOTOR_ID, MASTER_ANGLE_MOTOR_NAME),
            FOLLOWER_ELEVATOR_MOTOR = new TalonFXMotor(FOLLOWER_ELEVATOR_MOTOR_ID, FOLLOWER_ANGLE_MOTOR_NAME);
    static final SimpleSensor BEAM_BREAK = SimpleSensor.createDigitalSensor(BEAM_BREAK_PORT, BEAM_BREAK_NAME);

    private static final NeutralModeValue
            INTAKE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            FUNNEL_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            ELEVATOR_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue
            INTAKE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FUNNEL_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            MASTER_ELEVATOR_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_ELEVATOR_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean ELEVATOR_FOLLOWER_OPPOSES_MASTER = false;
    private static final double
            ELEVATOR_P = RobotHardwareStats.isSimulation() ? 300 : 0,
            ELEVATOR_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            ELEVATOR_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            ELEVATOR_KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            ELEVATOR_KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            ELEVATOR_KA = RobotHardwareStats.isSimulation() ? 0 : 0,
            ELEVATOR_KG = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final double
            ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY = RobotHardwareStats.isSimulation() ? 3 : 0,
            ELEVATOR_MOTION_MAGIC_ACCELERATION = RobotHardwareStats.isSimulation() ? 3 : 0,
            ELEVATOR_MOTION_MAGIC_JERK = ELEVATOR_MOTION_MAGIC_ACCELERATION * 10;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseVelocitySign;
    private static final double
            INTAKE_MOTOR_GEAR_RATIO = 1,
            FUNNEL_MOTOR_GEAR_RATIO = 1,
            ELEVATOR_MOTOR_GEAR_RATIO = 100;
    private static final ForwardLimitSourceValue FORWARD_LIMIT_SOURCE_VALUE = ForwardLimitSourceValue.LimitSwitchPin;
    private static final ReverseLimitSourceValue REVERSE_LIMIT_SOURCE_VALUE = ReverseLimitSourceValue.LimitSwitchPin;
    private static final ForwardLimitTypeValue FORWARD_LIMIT_TYPE_VALUE = ForwardLimitTypeValue.NormallyOpen;
    private static final ReverseLimitTypeValue REVERSE_LIMIT_TYPE_VALUE = ReverseLimitTypeValue.NormallyOpen;
    private static final double ELEVATOR_FORWARD_LIMIT_POSITION_ROTATIONS = 0;//TODO: Find
    static final boolean FOC_ENABLED = true;

    private static final int
            INTAKE_MOTOR_AMOUNT = 1,
            FUNNEL_MOTOR_AMOUNT = 1,
            ELEVATOR_MOTOR_AMOUNT = 2;
    private static final DCMotor
            INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(INTAKE_MOTOR_AMOUNT),
            FUNNEL_GEARBOX = DCMotor.getKrakenX60Foc(FUNNEL_MOTOR_AMOUNT),
            ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(ELEVATOR_MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            INTAKE_MASS_KILOGRAMS = 5,
            MINIMUM_OPENING_DISTANCE_METERS = 0,
            MAXIMUM_OPENING_DISTANCE_METERS = 0.35;
    private static final double ELEVATOR_DRUM_RADIUS_METERS = 0.02;
    private static final boolean SHOULD_SIMULATE_GRAVITY = false;
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
    private static final ElevatorSimulation
            ANGLE_MOTOR_SIMULATION = new ElevatorSimulation(
            ELEVATOR_GEARBOX,
            ELEVATOR_MOTOR_GEAR_RATIO,
            INTAKE_MASS_KILOGRAMS,
            ELEVATOR_DRUM_RADIUS_METERS,
            MINIMUM_OPENING_DISTANCE_METERS,
            MAXIMUM_OPENING_DISTANCE_METERS,
            SHOULD_SIMULATE_GRAVITY
    );
    private static final DoubleSupplier BEAM_BREAK_SIMULATION_VALUE_SUPPLIER = () -> SimulationObjectDetectionCameraIO.HAS_OBJECTS ? 1 : 0;

    static final SysIdRoutine.Config ELEVATOR_SYSID_CONFIG = new SysIdRoutine.Config(
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
    static final ElevatorMechanism2d
            ELEVATOR_MECHANISM = new ElevatorMechanism2d(
            "CoralElevatorMechanism",
            MAXIMUM_OPENING_DISTANCE_METERS,
            MINIMUM_OPENING_DISTANCE_METERS,
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
    static final double ELEVATOR_DRUM_DIAMETER_METERS = ELEVATOR_DRUM_RADIUS_METERS * 2;
    static final double
            POSITION_TOLERANCE_METERS = 0.01,
            POSITION_TOLERANCE_ROTATIONS = Conversions.distanceToRotations(POSITION_TOLERANCE_METERS, ELEVATOR_DRUM_DIAMETER_METERS);

    static {
        configureIntakeMotor();
        configureFunnelMotor();
        configureMasterElevatorMotor();
        configureFollowerElevatorMotor();
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

    private static void configureMasterElevatorMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = ELEVATOR_MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = MASTER_ELEVATOR_MOTOR_INVERTED_VALUE;
        config.Feedback.RotorToSensorRatio = ELEVATOR_MOTOR_GEAR_RATIO;

        config.Slot0.kP = ELEVATOR_P;
        config.Slot0.kI = ELEVATOR_I;
        config.Slot0.kD = ELEVATOR_D;
        config.Slot0.kS = ELEVATOR_KS;
        config.Slot0.kV = ELEVATOR_KV;
        config.Slot0.kA = ELEVATOR_KA;
        config.Slot0.kG = ELEVATOR_KG;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

        config.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ELEVATOR_MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ELEVATOR_MOTION_MAGIC_JERK;

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitType = FORWARD_LIMIT_TYPE_VALUE;
        config.HardwareLimitSwitch.ForwardLimitSource = FORWARD_LIMIT_SOURCE_VALUE;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = ELEVATOR_FORWARD_LIMIT_POSITION_ROTATIONS;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitType = REVERSE_LIMIT_TYPE_VALUE;
        config.HardwareLimitSwitch.ReverseLimitSource = REVERSE_LIMIT_SOURCE_VALUE;

        MASTER_ELEVATOR_MOTOR.applyConfiguration(config);
        MASTER_ELEVATOR_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        MASTER_ELEVATOR_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_ELEVATOR_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_ELEVATOR_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_ELEVATOR_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_ELEVATOR_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureFollowerElevatorMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.MotorOutput.NeutralMode = ELEVATOR_MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = FOLLOWER_ELEVATOR_MOTOR_INVERTED_VALUE;

        FOLLOWER_ELEVATOR_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_ELEVATOR_MOTOR_ID, ELEVATOR_FOLLOWER_OPPOSES_MASTER);
        FOLLOWER_ELEVATOR_MOTOR.setControl(followerRequest);
    }

    private static void configureBeamBreak() {
        BEAM_BREAK.setSimulationSupplier(BEAM_BREAK_SIMULATION_VALUE_SUPPLIER);
    }

    public enum CoralIntakeState {
        COLLECT(12, 12, 0.1),
        LOAD(-12, -12, 0),
        EJECT(-12, -12, 0.1),
        RETRACT(0, 0, 0),
        REST(0, 0, 0);

        public final double
                targetIntakeVoltage,
                targetFunnelVoltage;
        public final double targetPositionRotations;

        CoralIntakeState(double targetIntakeVoltage, double targetFunnelVoltage, double targetPositionRotations) {
            this.targetIntakeVoltage = targetIntakeVoltage;
            this.targetFunnelVoltage = targetFunnelVoltage;
            this.targetPositionRotations = targetPositionRotations;
        }
    }
}