package frc.trigon.robot.subsystems.algaeintake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

public class AlgaeIntakeConstants {
    private static final int
            INTAKE_MOTOR_ID = 13,
            ANGLE_MOTOR_ID = 14,
            ENCODER_ID = 14;
    private static final String
            INTAKE_MOTOR_NAME = "AlgaeIntakeMotor",
            ANGLE_MOTOR_NAME = "AlgaeIntakeAngleMotor",
            ENCODER_NAME = "AlgaeIntakeAngleEncoder";
    static final TalonFXMotor
            INTAKE_MOTOR = new TalonFXMotor(INTAKE_MOTOR_ID, INTAKE_MOTOR_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    private static final InvertedValue
            INTAKE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive,
            ANGLE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue
            INTAKE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast,
            ANGLE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
    private static final double
            INTAKE_MOTOR_GEAR_RATIO = 1,
            ANGLE_MOTOR_GEAR_RATIO = 200;
    private static final double
            ANGLE_P = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KA = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KG = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final double
            MOTION_MAGIC_ACCELERATION = 100,
            MOTION_MAGIC_VELOCITY = 100,
            MOTION_MAGIC_JERK = 0.5;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    private static final Rotation2d ENCODER_MAGNET_OFFSET = Rotation2d.fromRotations(0);
    private static final double ENCODER_DISCONTINUITY_POINT = 0.5;
    static final boolean ENABLE_FOC = true;

    private static final int
            INTAKE_MOTOR_AMOUNT = 1,
            ANGLE_MOTOR_AMOUNT = 1;
    private static final DCMotor
            INTAKE_MOTOR_GEARBOX = DCMotor.getKrakenX60(INTAKE_MOTOR_AMOUNT),
            ANGLE_MOTOR_GEARBOX = DCMotor.getKrakenX60(ANGLE_MOTOR_AMOUNT);
    private static final double INTAKE_MOTOR_MOMENT_OF_INERTIA = 0.0003;
    private static final double
            INTAKE_LENGTH_METERS = 0.5,
            INTAKE_MASS_KILOGRAMS = 0.5;
    private static final Rotation2d
            INTAKE_MAXIMUM_ANGLE = Rotation2d.fromDegrees(90),
            INTAKE_MINIMUM_ANGLE = Rotation2d.fromDegrees(0);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SimpleMotorSimulation INTAKE_MOTOR_SIMULATION = new SimpleMotorSimulation(
            INTAKE_MOTOR_GEARBOX,
            INTAKE_MOTOR_GEAR_RATIO,
            INTAKE_MOTOR_MOMENT_OF_INERTIA
    );
    private static final SingleJointedArmSimulation ANGLE_MOTOR_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_MOTOR_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            INTAKE_MASS_KILOGRAMS,
            INTAKE_LENGTH_METERS,
            INTAKE_MAXIMUM_ANGLE,
            INTAKE_MINIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(5).per(Units.Second),
            Units.Volts.of(12),
            Units.Second.of(1000)
    );

    private static final double INTAKE_MOTOR_MAXIMUM_SPEED = 12;
    private static final Color ANGLE_MOTOR_MECHANSIM_COLOR = Color.kGreen;
    static final SpeedMechanism2d INTAKE_MECHANISM = new SpeedMechanism2d("AlgaeIntakeMechanism", INTAKE_MOTOR_MAXIMUM_SPEED);
    static final SingleJointedArmMechanism2d ANGLE_MECHANISM = new SingleJointedArmMechanism2d("AlgaeIntakeAngleMechanism", ANGLE_MOTOR_MECHANSIM_COLOR);
    static final Pose3d ALGAE_INTAKE_VISUALIZATION_ORIGIN_POINT = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(2);

    static {
        configureIntakeMotor();
        configureAngleMotor();
        configureEncoder();
    }

    private static void configureIntakeMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = INTAKE_MOTOR_INVERTED;
        config.MotorOutput.NeutralMode = INTAKE_MOTOR_NEUTRAL_MODE;

        INTAKE_MOTOR.applyConfiguration(config);
        INTAKE_MOTOR.setPhysicsSimulation(INTAKE_MOTOR_SIMULATION);

        INTAKE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = ANGLE_MOTOR_INVERTED;
        config.MotorOutput.NeutralMode = ANGLE_MOTOR_NEUTRAL_MODE;

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;
        config.Slot0.kG = ANGLE_KG;

        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET.getRotations();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ENCODER_DISCONTINUITY_POINT;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(ANGLE_MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
    }

    public enum AlgaeIntakeState {
        COLLECT(Rotation2d.fromDegrees(0), 5),
        EJECT(Rotation2d.fromDegrees(0), -5),
        REST(Rotation2d.fromDegrees(90), 0);

        public final Rotation2d angle;
        public final double voltage;

        AlgaeIntakeState(Rotation2d angle, double voltage) {
            this.angle = angle;
            this.voltage = voltage;
        }
    }
}