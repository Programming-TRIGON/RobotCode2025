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
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
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
            GRIPPING_MOTOR_ID = 14,
            ANGLE_MOTOR_ID = 15,
            ANGLE_ENCODER_ID = 15,
            LASER_CAN_ID = 14;
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
            ANGLE_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue
            GRIPPER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            ANGLE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double GRIPPING_MOTOR_GEAR_RATIO = 4;
    static final double
            ANGLE_MOTOR_GEAR_RATIO = 34.2857;
    private static final double
            ANGLE_P = RobotHardwareStats.isSimulation() ? 100 : 20,
            ANGLE_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            ANGLE_KS = RobotHardwareStats.isSimulation() ? 0 : 0.42701,
            ANGLE_KV = RobotHardwareStats.isSimulation() ? 0 : 2.3896,
            ANGLE_KA = RobotHardwareStats.isSimulation() ? 0 : 0.34116,
            ANGLE_KG = RobotHardwareStats.isSimulation() ? 0 : 0.20883;
    private static final double
            ANGLE_MOTION_MAGIC_CRUISE_VELOCITY = RobotHardwareStats.isSimulation() ? 5 : 5,
            ANGLE_MOTION_MAGIC_ACCELERATION = RobotHardwareStats.isSimulation() ? 5 : 5,
            ANGLE_MOTION_MAGIC_JERK = ANGLE_MOTION_MAGIC_ACCELERATION * 10;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseVelocitySign;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
    private static final Rotation2d
            ANGLE_REVERSE_SOFT_LIMIT_THRESHOLD = Rotation2d.fromRotations(-0.28),
            ANGLE_FORWARD_SOFT_LIMIT_THRESHOLD = Rotation2d.fromDegrees(55).minus(Rotation2d.fromRotations(0.14092));
    private static final FeedbackSensorSourceValue ANGLE_ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final SensorDirectionValue ANGLE_ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final double
            ANGLE_ENCODER_GRAVITY_OFFSET = 0.43 + 0.25 - 0.14092,
            ANGLE_ENCODER_DISCONTINUITY_POINT = 0.5;
    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET = 0.14092;
    private static final int
            LASER_CAN_DETECTION_REGION_START_X_COORDINATE = 6,
            LASER_CAN_DETECTION_REGION_START_Y_COORDINATE = 6,
            LASER_CAN_DETECTION_REGION_END_X_COORDINATE = 12,
            LASER_CAN_DETECTION_REGION_END_Y_COORDINATE = 12;
    private static final LaserCan.RangingMode LASER_CAN_RANGING_MODE = LaserCan.RangingMode.SHORT;
    private static final LaserCan.TimingBudget LASER_CAN_LOOP_TIME = LaserCan.TimingBudget.TIMING_BUDGET_33MS;
    static final boolean FOC_ENABLED = true;

    private static final int
            NUMBER_OF_GRIPPING_MOTORS = 1,
            NUMBER_OF_ARM_MOTORS = 1;
    private static final DCMotor
            GRIPPING_GEARBOX = DCMotor.getFalcon500Foc(NUMBER_OF_GRIPPING_MOTORS),
            ARM_GEARBOX = DCMotor.getKrakenX60(NUMBER_OF_ARM_MOTORS);
    private static final double
            ARM_LENGTH_METERS = 0.24,
            ARM_MASS_KILOGRAMS = 3;
    private static final Rotation2d
            ARM_MINIMUM_ANGLE = Rotation2d.fromDegrees(-55),
            ARM_MAXIMUM_ANGLE = Rotation2d.fromDegrees(120);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final DoubleSupplier LASER_CAN_SIMULATION_SUPPLIER = () -> SimulationFieldHandler.isCoralInGripper() && SimulationFieldHandler.isHoldingCoral() ? 1 : 10000;
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
            Units.Volts.of(0.25).per(Units.Seconds),
            Units.Volts.of(1),
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
            new Translation3d(-0.224, 0, 0.8622),
            new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(62), 0)
    );
    static final Transform3d
            ELEVATOR_TO_GRIPPER = new Transform3d(
            ElevatorConstants.FIRST_STAGE_VISUALIZATION_ORIGIN_POINT,
            GRIPPER_VISUALIZATION_ORIGIN_POINT
    ),
            GRIPPER_TO_CORAL_RELEASE = new Transform3d(
                    new Translation3d(0.3, 0, -0.1),
                    new Rotation3d(0, 0, 0)
            ),
            GRIPPER_TO_HELD_CORAL = new Transform3d(
                    new Translation3d(0.05, 0, -0.1),
                    new Rotation3d(0, 0, 0)
            );

    static final double WHEEL_DIAMETER_METERS = edu.wpi.first.math.util.Units.inchesToMeters(2.5);
    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(2);
    static final double SCORE_IN_REEF_VOLTAGE = -8;
    static final double MINIMUM_VOLTAGE_FOR_EJECTING = -3;
    static final Rotation2d MINIMUM_OPEN_FOR_ELEVATOR_ANGLE = Rotation2d.fromDegrees(-55);
    static final double GAME_PIECE_DETECTION_THRESHOLD_MILLIMETERS = 10;

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
        GRIPPING_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
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
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;
        config.Slot0.kG = ANGLE_KG;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

        config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ANGLE_MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ANGLE_MOTION_MAGIC_JERK;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ANGLE_REVERSE_SOFT_LIMIT_THRESHOLD.getRotations();

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ANGLE_FORWARD_SOFT_LIMIT_THRESHOLD.getRotations();

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ARM_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = ANGLE_ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
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
        REST(Rotation2d.fromDegrees(-60), 0),
        PREPARE_FOR_EJECTING(Rotation2d.fromDegrees(45), 0),
        EJECT(Rotation2d.fromDegrees(45), -5),
        PREPARE_L4(Rotation2d.fromDegrees(55), 0),
        PREPARE_L3_OR_L2(Rotation2d.fromDegrees(55), 0),
        PREPARE_L1(Rotation2d.fromDegrees(55), 0),
        LOAD_CORAL(Rotation2d.fromDegrees(-50), 3),
        PREPARE_FOR_LOADING_CORAL(Rotation2d.fromDegrees(-50), 0),
        COLLECT_FROM_FEEDER(Rotation2d.fromDegrees(90), -3),
        OPEN_FOR_ELEVATOR_MINIMUM(Rotation2d.fromDegrees(-54), 0);

        final Rotation2d targetAngle;
        final double targetGripperVoltage;

        GripperState(Rotation2d targetAngle, double targetVoltage) {
            this.targetAngle = targetAngle;
            this.targetGripperVoltage = targetVoltage;
        }
    }
}