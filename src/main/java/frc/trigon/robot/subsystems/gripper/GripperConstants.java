package frc.trigon.robot.subsystems.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import org.trigon.hardware.grapple.lasercan.LaserCAN;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;


public class GripperConstants {
    private static final int
            GRIPPER_MOTOR_ID = 15,
            ANGLE_MOTOR_ID = 16,
            ANGLE_ENCODER_ID = 16,
            LASER_CAN_ID = 0;
    private static final String
            GRIPPER_MOTOR_NAME = "GripperMotor",
            ANGLE_MOTOR_NAME = "AngleMotor",
            ENCODER_NAME = "Encoder",
            LASER_CAN_NAME = "GripperLaserCAN";
    static final TalonFXMotor GRIPPER_MOTOR = new TalonFXMotor(GRIPPER_MOTOR_ID, GRIPPER_MOTOR_NAME);
    static final TalonFXMotor ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ENCODER_NAME);
    static final LaserCAN LASER_CAN = new LaserCAN(LASER_CAN_ID, LASER_CAN_NAME);

    private static final double
            GRIPPER_MOTOR_GEAR_RATIO = 1,
            ANGLE_MOTOR_GEAR_RATIO = 1;
    private static final InvertedValue
            GRIPPER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            ANGLE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue
            GRIPPER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            ANGLE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final FeedbackSensorSourceValue ANGLE_ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final SensorDirectionValue ANGLE_ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final double
            ANGLE_ENCODER_MAGNET_OFFSET_VALUE = 0,
            ANGLE_ENCODER_DISCONTINUITY_POINT = 0.5;
    private static final int
            LASER_CAN_DETECTION_REGION_START_X_COORDINATE = 6,
            LASER_CAN_DETECTION_REGION_START_Y_COORDINATE = 6,
            LASER_CAN_DETECTION_REGION_END_X_COORDINATE = 11,
            LASER_CAN_DETECTION_REGION_END_Y_COORDINATE = 11;
    private static final LaserCan.RangingMode LASER_CAN_RANGING_MODE = LaserCan.RangingMode.SHORT;
    private static final LaserCan.TimingBudget LASER_CAN_LOOP_TIME = LaserCan.TimingBudget.TIMING_BUDGET_33MS;
    private static final DoubleSupplier LASER_CAN_SIMULATION_SUPPLIER = () -> 1;
    static final double LASER_CAN_THRESHOLD = 10;
    static final boolean FOC_ENABLED = true;

    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    private static final int
            NUMBER_OF_GRIPPER_MOTORS = 1,
            NUMBER_OF_ARM_MOTORS = 1;
    private static final DCMotor
            GRIPPER_GEARBOX = DCMotor.getKrakenX60(NUMBER_OF_GRIPPER_MOTORS),
            ARM_GEARBOX = DCMotor.getKrakenX60(NUMBER_OF_ARM_MOTORS);
    private static final double
            ARM_LENGTH_METERS = 1,
            ARM_MASS_KILOGRAMS = 1;
    private static final Rotation2d
            ARM_MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            ARM_MAXIMUM_ANGLE = Rotation2d.fromDegrees(180);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final SimpleMotorSimulation GRIPPER_SIMULATION = new SimpleMotorSimulation(
            GRIPPER_GEARBOX,
            GRIPPER_MOTOR_GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            ARM_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            ARM_LENGTH_METERS,
            ARM_MASS_KILOGRAMS,
            ARM_MINIMUM_ANGLE,
            ARM_MAXIMUM_ANGLE,
            SIMULATE_GRAVITY
    );

    static final SpeedMechanism2d GRIPPER_MECHANISM = new SpeedMechanism2d(
            "GripperMechanism",
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static final SingleJointedArmMechanism2d ARM_MECHANISM = new SingleJointedArmMechanism2d(
            "GripperAngleMechanism",
            ARM_LENGTH_METERS,
            Color.kRed
    );

    static {
        configureGripperMotor();
        configureAngleMotor();
        configureEncoder();
        configureLaserCAN();
    }

    private static void configureGripperMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = GRIPPER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = GRIPPER_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.RotorToSensorRatio = GRIPPER_MOTOR_GEAR_RATIO;

        GRIPPER_MOTOR.applyConfiguration(config);
        GRIPPER_MOTOR.setPhysicsSimulation(GRIPPER_SIMULATION);

        GRIPPER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        GRIPPER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = ANGLE_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = ANGLE_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;

        config.Feedback.FeedbackRemoteSensorID = ANGLE_ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ANGLE_ENCODER_TYPE;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ARM_SIMULATION);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = ANGLE_ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_MAGNET_OFFSET_VALUE;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ANGLE_ENCODER_DISCONTINUITY_POINT;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ANGLE_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void configureLaserCAN() {
        try {
            LASER_CAN.setRegionOfInterest(LASER_CAN_DETECTION_REGION_START_X_COORDINATE, LASER_CAN_DETECTION_REGION_START_Y_COORDINATE, LASER_CAN_DETECTION_REGION_END_X_COORDINATE, LASER_CAN_DETECTION_REGION_END_Y_COORDINATE);
            LASER_CAN.setRangingMode(LASER_CAN_RANGING_MODE);
            LASER_CAN.setLoopTime(LASER_CAN_LOOP_TIME);
            LASER_CAN.setSimulationSupplier(LASER_CAN_SIMULATION_SUPPLIER);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }

    public enum GripperState {
        STOP(0, Rotation2d.fromDegrees(0)),
        RELEASE(-4, Rotation2d.fromDegrees(90)),
        GRAB(4, Rotation2d.fromDegrees(180));

        final double targetGripperVoltage;
        final Rotation2d targetAngle;

        GripperState(double targetVoltage, Rotation2d targetAngle) {
            this.targetGripperVoltage = targetVoltage;
            this.targetAngle = targetAngle;
        }
    }
}