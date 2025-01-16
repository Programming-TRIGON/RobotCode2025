package frc.trigon.robot.subsystems.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import org.trigon.hardware.RobotHardwareStats;
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
            GRIPPING_MOTOR_ID = 17,
            ANGLE_MOTOR_ID = 18,
            ANGLE_ENCODER_ID = 18,
            LASER_CAN_ID = 17;
    private static final String
            GRIPPING_MOTOR_NAME = "GripperGrippingMotor",
            ANGLE_MOTOR_NAME = "GripperAngleMotor",
            ANGLE_ENCODER_NAME = "GripperAngleEncoder",
            LASER_CAN_NAME = "GripperLaserCAN";
    static final TalonFXMotor
            GRIPPING_MOTOR = new TalonFXMotor(GRIPPING_MOTOR_ID, GRIPPING_MOTOR_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME);
    static final LaserCAN LASER_CAN = new LaserCAN(LASER_CAN_ID, LASER_CAN_NAME);

    private static final InvertedValue
            GRIPPING_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            ANGLE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue
            GRIPPER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            ANGLE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            GRIPPING_MOTOR_GEAR_RATIO = 1,
            ANGLE_MOTOR_GEAR_RATIO = 100;
    private static final double
            ANGLE_P = RobotHardwareStats.isSimulation() ? 140 : 0,
            ANGLE_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KA = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KG = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final double
            ANGLE_MOTION_MAGIC_CRUISE_VELOCITY = RobotHardwareStats.isSimulation() ? 5 : 0,
            ANGLE_MOTION_MAGIC_ACCELERATION = RobotHardwareStats.isSimulation() ? 5 : 0,
            ANGLE_MOTION_MAGIC_JERK = ANGLE_MOTION_MAGIC_ACCELERATION * 10;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseVelocitySign;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
    private static final ForwardLimitSourceValue FORWARD_LIMIT_SOURCE_VALUE = ForwardLimitSourceValue.LimitSwitchPin;
    private static final ReverseLimitSourceValue REVERSE_LIMIT_SOURCE_VALUE = ReverseLimitSourceValue.LimitSwitchPin;
    private static final ForwardLimitTypeValue FORWARD_LIMIT_TYPE_VALUE = ForwardLimitTypeValue.NormallyOpen;
    private static final ReverseLimitTypeValue REVERSE_LIMIT_TYPE_VALUE = ReverseLimitTypeValue.NormallyOpen;

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
    static final boolean FOC_ENABLED = true;

    private static final int
            NUMBER_OF_GRIPPING_MOTORS = 1,
            NUMBER_OF_ARM_MOTORS = 1;
    private static final DCMotor
            GRIPPING_GEARBOX = DCMotor.getKrakenX60(NUMBER_OF_GRIPPING_MOTORS),
            ARM_GEARBOX = DCMotor.getKrakenX60(NUMBER_OF_ARM_MOTORS);
    private static final double
            ARM_LENGTH_METERS = 1,
            ARM_MASS_KILOGRAMS = 1;
    private static final Rotation2d
            ARM_MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            ARM_MAXIMUM_ANGLE = Rotation2d.fromDegrees(180);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final DoubleSupplier LASER_CAN_SIMULATION_SUPPLIER = () -> 1;
    private static final SimpleMotorSimulation GRIPPING_SIMULATION = new SimpleMotorSimulation(
            GRIPPING_GEARBOX,
            GRIPPING_MOTOR_GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            ARM_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            ARM_LENGTH_METERS,
            ARM_MASS_KILOGRAMS,
            ARM_MINIMUM_ANGLE,
            ARM_MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Seconds),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d GRIPPING_MECHANISM = new SpeedMechanism2d(
            "GripperGrippingMechanism",
            MAXIMUM_DISPLAYABLE_VELOCITY
    );
    static final SingleJointedArmMechanism2d ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            "GripperAngleMechanism",
            ARM_LENGTH_METERS,
            Color.kRed
    );

    private static final Pose3d GRIPPER_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(-0.22, 0, 0.858),
            new Rotation3d(0, 0, 0)
    );
    static final Transform3d
            ELEVATOR_TO_GRIPPER = new Transform3d(
            ElevatorConstants.FIRST_STAGE_VISUALIZATION_ORIGIN_POINT,
            GRIPPER_VISUALIZATION_ORIGIN_POINT
    ),
            GRIPPER_TO_CORAL_RELEASE = new Transform3d(
                    new Translation3d(0.117, 0, -0.185),
                    new Rotation3d(0, 0, 0)
            );
    static final Rotation2d POSITION_TOLERANCE_DEGREES = Rotation2d.fromDegrees(2);

    static {
        configureGrippingMotor();
        configureAngleMotor();
        configureEncoder();
        configureLaserCAN();
    }

    private static void configureGrippingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = GRIPPING_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = GRIPPER_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.RotorToSensorRatio = GRIPPING_MOTOR_GEAR_RATIO;

        GRIPPING_MOTOR.applyConfiguration(config);
        GRIPPING_MOTOR.setPhysicsSimulation(GRIPPING_SIMULATION);

        GRIPPING_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        GRIPPING_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
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

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kA = ANGLE_KA;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kG = ANGLE_KG;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

        config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ANGLE_MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ANGLE_MOTION_MAGIC_JERK;

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitSource = FORWARD_LIMIT_SOURCE_VALUE;
        config.HardwareLimitSwitch.ForwardLimitType = FORWARD_LIMIT_TYPE_VALUE;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitSource = REVERSE_LIMIT_SOURCE_VALUE;
        config.HardwareLimitSwitch.ReverseLimitType = REVERSE_LIMIT_TYPE_VALUE;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ARM_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
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

    static final double GAME_PIECE_DETECTION_THRESHOLD_MILLIMETERS = 10;

    public enum GripperState {
        REST(Rotation2d.fromDegrees(0), 0),
        EJECT(Rotation2d.fromDegrees(45), -3),
        SCORE_L4(Rotation2d.fromDegrees(110), 3),
        SCORE_L3_OR_L2(Rotation2d.fromDegrees(120), 3),
        SCORE_L1(Rotation2d.fromDegrees(45), 3),
        LOAD_CORAL(Rotation2d.fromDegrees(-56), -3),
        COLLECT_FROM_FEEDER(Rotation2d.fromDegrees(90), -3);


        final Rotation2d targetAngle;
        final double targetGripperVoltage;

        GripperState(Rotation2d targetAngle, double targetVoltage) {
            this.targetAngle = targetAngle;
            this.targetGripperVoltage = targetVoltage;
        }
    }
}