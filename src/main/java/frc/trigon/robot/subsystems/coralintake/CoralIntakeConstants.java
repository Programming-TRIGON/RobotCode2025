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
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(9, "CoralIntakeMotor"),
            FUNNEL_MOTOR = new TalonFXMotor(10, "CoralFunnelMotor"),
            ANGLE_MOTOR = new TalonFXMotor(11, "CoralAngleMotor");
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(11, "CoralAngleEncoder");
    static final SimpleSensor
            BEAM_BREAK = SimpleSensor.createDigitalSensor(0, "CoralBeamBreak"),
            DISTANCE_SENSOR = SimpleSensor.createDutyCycleSensor(1, "CoralDistanceSensor");

    private static final double
            INTAKE_MOTOR_GEAR_RATIO = 1.3,
            FUNNEL_MOTOR_GEAR_RATIO = 4;
    static final double ANGLE_MOTOR_GEAR_RATIO = 73;

    static final double ANGLE_ENCODER_POSITION_OFFSET_VALUE = -0.00917;

    static final boolean FOC_ENABLED = true;

    private static final SimpleMotorSimulation
            INTAKE_SIMULATION = new SimpleMotorSimulation(
            DCMotor.getFalcon500Foc(1),
            INTAKE_MOTOR_GEAR_RATIO,
            0.003
    ),
            FUNNEL_SIMULATION = new SimpleMotorSimulation(
                    DCMotor.getFalcon500Foc(1),
                    FUNNEL_MOTOR_GEAR_RATIO,
                    0.003
            );
    private static final SingleJointedArmSimulation
            ANGLE_MOTOR_SIMULATION = new SingleJointedArmSimulation(
            DCMotor.getKrakenX60Foc(1),
            ANGLE_MOTOR_GEAR_RATIO,
            0.44,
            8,
            Rotation2d.fromDegrees(-48),
            Rotation2d.fromDegrees(150),
            true
    );
    private static final DoubleSupplier BEAM_BREAK_SIMULATION_VALUE_SUPPLIER = () -> SimulationFieldHandler.isHoldingCoral() && !SimulationFieldHandler.isCoralInGripper() ? 1 : 0;

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
    static final SpeedMechanism2d
            INTAKE_MECHANISM = new SpeedMechanism2d(
            "CoralIntakeMechanism",
            12
    ),
            FUNNEL_MECHANISM = new SpeedMechanism2d(
                    "CoralFunnelMechanism",
                    12
            );
    static final SingleJointedArmMechanism2d
            ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            "CoralAngleMechanism",
            0.44,
            Color.kRed
    );

    public static final double
            COLLECTION_LEDS_BLINKING_SPEED = 0.5,
            COLLECTION_RUMBLE_DURATION_SECONDS = 0.7,
            COLLECTION_RUMBLE_POWER = 1;
    static final BooleanEvent CORAL_COLLECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            BEAM_BREAK::getBinaryValue
    ).debounce(0.65);
    static final BooleanEvent EARLY_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> RobotHardwareStats.isSimulation() ? SimulationFieldHandler.isHoldingCoral() : DISTANCE_SENSOR.getScaledValue() < 17
    ).debounce(0.06);

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.5);

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
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 75 : 20;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 1.6663 : 0.5;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.074947 : 0.12;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 8.7544 : 8.55;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.27712 : 0.41506;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 12 / config.Slot0.kV : 1;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 6 : 10;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.128662 - ANGLE_ENCODER_POSITION_OFFSET_VALUE;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotation2d.fromDegrees(143.8).minus(Rotation2d.fromRotations(ANGLE_ENCODER_POSITION_OFFSET_VALUE)).getRotations();

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
        config.MagnetSensor.MagnetOffset = -0.31585;
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
        DISTANCE_SENSOR.setScalingConstants(0.0002, -200);
        DISTANCE_SENSOR.setSimulationSupplier(() -> 1000);
    }

    public enum CoralIntakeState {
        LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE(-3, -1, Rotation2d.fromDegrees(143.5)),
        LOAD_CORAL_TO_GRIPPER_NOT_SEEING_GAME_PIECE(-3, 0, LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE.targetAngle),
        PREPARE_FOR_LOADING_TO_GRIPPER_WHILE_GAME_PIECE_NOT_DETECTED(6, 2, LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE.targetAngle),
        UNLOAD_CORAL_FROM_GRIPPER(6, 2, Rotation2d.fromDegrees(141)),
        CENTER_CORAL(6, 2, Rotation2d.fromDegrees(143.5)),
        COLLECT_FROM_FLOOR(6, 2, Rotation2d.fromDegrees(-46)),
        COLLECT_FROM_FEEDER(6, 2, Rotation2d.fromDegrees(90)),
        EJECT(-3, -1, Rotation2d.fromDegrees(45)),
        REST(0, 0, Rotation2d.fromDegrees(143.5)),
        SCORE_L1(-3, -1, Rotation2d.fromDegrees(45));

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