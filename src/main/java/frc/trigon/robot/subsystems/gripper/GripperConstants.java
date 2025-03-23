package frc.trigon.robot.subsystems.gripper;

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
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;


public class GripperConstants {
    public static final int
            GRIPPING_MOTOR_ID = 14,
            ANGLE_MOTOR_ID = 15,
            ANGLE_ENCODER_ID = 15,
            BEAM_BREAK_CHANNEL = 5;
    private static final String
            GRIPPING_MOTOR_NAME = "GripperGrippingMotor",
            ANGLE_MOTOR_NAME = "GripperAngleMotor",
            ANGLE_ENCODER_NAME = "GripperAngleEncoder",
            BEAM_BREAK_NAME = "GripperBeamBreak";
    static final TalonFXMotor
            GRIPPING_MOTOR = new TalonFXMotor(GRIPPING_MOTOR_ID, GRIPPING_MOTOR_NAME),
            ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(ANGLE_ENCODER_ID, ANGLE_ENCODER_NAME);
    static final SimpleSensor BEAM_BREAK = SimpleSensor.createDigitalSensor(BEAM_BREAK_CHANNEL, BEAM_BREAK_NAME);

    static final double ANGLE_MOTOR_GEAR_RATIO = 20.5;
    private static final double GRIPPING_MOTOR_GEAR_RATIO = 4;
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = -0.12331 + 0.10867 + Conversions.degreesToRotations(60) - 0.063537;
    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET = RobotHardwareStats.isSimulation() ? 0 : -0.044444 + Conversions.degreesToRotations(60) - ANGLE_ENCODER_GRAVITY_OFFSET;
    static final double
            DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 5 : 6,
            DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 5 : 6.5;
    static final boolean FOC_ENABLED = true;

    private static final int
            GRIPPING_MOTOR_AMOUNT = 1,
            ANGLE_MOTOR_AMOUNT = 1;
    private static final DCMotor
            GRIPPING_GEARBOX = DCMotor.getFalcon500Foc(GRIPPING_MOTOR_AMOUNT),
            ANGLE_GEARBOX = DCMotor.getKrakenX60Foc(ANGLE_MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            GRIPPER_LENGTH_METERS = 0.24,
            GRIPPER_MASS_KILOGRAMS = 3;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-55),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(120);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SimpleMotorSimulation GRIPPING_SIMULATION = new SimpleMotorSimulation(
            GRIPPING_GEARBOX,
            GRIPPING_MOTOR_GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            GRIPPER_LENGTH_METERS,
            GRIPPER_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );
    private static final DoubleSupplier BEAM_BREAK_SIMULATION_SUPPLIER = () -> SimulationFieldHandler.isCoralInGripper() && SimulationFieldHandler.isHoldingCoral() ? 1 : 0;

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.5).per(Units.Seconds),
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
            GRIPPER_LENGTH_METERS,
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
    static final double SCORE_IN_REEF_FOR_AUTO_VOLTAGE = -5;
    static final double MINIMUM_VOLTAGE_FOR_EJECTING = -3;
    static final Rotation2d MINIMUM_OPEN_FOR_ELEVATOR_ANGLE = Rotation2d.fromDegrees(-34);
    private static final double COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS = 0.10;
    static final BooleanEvent COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            BEAM_BREAK::getBinaryValue
    ).debounce(COLLECTION_DETECTION_DEBOUNCE_TIME_SECONDS);
    static final double SCORE_L4_FAR_DISTANCE_METERS = 0.1;

    static {
        configureGrippingMotor();
        configureAngleMotor();
        configureEncoder();
        configureBeamBreak();
    }

    private static void configureGrippingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.RotorToSensorRatio = GRIPPING_MOTOR_GEAR_RATIO;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;

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

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.RotorToSensorRatio = ANGLE_MOTOR_GEAR_RATIO;

        config.Feedback.FeedbackRemoteSensorID = ANGLE_MOTOR.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 100 : 60;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 3;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0.31059;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 1.4312;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0 : 0.44675;

        config.Slot1.kP = RobotHardwareStats.isSimulation() ? 100 : 35;
        config.Slot1.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot1.kD = RobotHardwareStats.isSimulation() ? 0 : 3.1312;
        config.Slot1.kS = config.Slot0.kS;
        config.Slot1.kV = config.Slot0.kV;
        config.Slot1.kA = config.Slot0.kA;
        config.Slot1.kG = RobotHardwareStats.isSimulation() ? config.Slot0.kG : 0.39472;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Rotation2d.fromDegrees(-59.645756).getRotations() - POSITION_OFFSET_FROM_GRAVITY_OFFSET;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotation2d.fromDegrees(120).getRotations() - POSITION_OFFSET_FROM_GRAVITY_OFFSET;

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ARM_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ANGLE_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void configureBeamBreak() {
        BEAM_BREAK.setSimulationSupplier(BEAM_BREAK_SIMULATION_SUPPLIER);
    }

    public enum GripperState {
        REST(Rotation2d.fromDegrees(-56), 0, 1),
        EJECT(Rotation2d.fromDegrees(55), -3, 1),
        EJECT_UPWARDS(Rotation2d.fromDegrees(107), -3, 1),
        SCORE_L4_CLOSE(Rotation2d.fromDegrees(45), -6, 1),
        SCORE_L4_FAR(Rotation2d.fromDegrees(60), -6, 1),
        SCORE_L3_OR_L2(Rotation2d.fromDegrees(60), SCORE_L4_CLOSE.targetGripperVoltage, 1),
        SCORE_L1(Rotation2d.fromDegrees(93), -3, 1),
        LOAD_CORAL(Rotation2d.fromDegrees(-56), 11, 1),
        UNLOAD_CORAL(Rotation2d.fromDegrees(-50), -3, 1),
        COLLECT_ALGAE_FROM_REEF(Rotation2d.fromDegrees(30), -50, 1),
        COLLECT_ALGAE_FROM_LOLLIPOP(Rotation2d.fromDegrees(-35), -50, 1),
        HOLD_ALGAE(Rotation2d.fromDegrees(40), -40, 0.3),
        SCORE_ALGAE_IN_NET(Rotation2d.fromDegrees(60), 11, 1),
        PREPARE_FOR_SCORING_ALGAE_IN_NET(Rotation2d.fromDegrees(100), COLLECT_ALGAE_FROM_REEF.targetGripperVoltage, 0.3),
        SCORE_ALGAE_IN_PROCESSOR(Rotation2d.fromDegrees(-35), 11, 1),
        PREPARE_FOR_SCORING_ALGAE_IN_PROCESSOR(Rotation2d.fromDegrees(-35), COLLECT_ALGAE_FROM_REEF.targetGripperVoltage, 0.3),
        AFTER_ELEVATOR_OPEN_POSITION(Rotation2d.fromDegrees(0), 0, 1),
        COLLECT_CORAL_FROM_FEEDER(Rotation2d.fromDegrees(90), 8, 1),
        OPEN_FOR_NOT_HITTING_REEF(Rotation2d.fromDegrees(107), 0.2, 1);

        final Rotation2d targetAngle;
        final double targetGripperVoltage;
        final double speedScalar;

        GripperState(Rotation2d targetAngle, double targetVoltage, double speedScalar) {
            this.targetAngle = targetAngle;
            this.targetGripperVoltage = targetVoltage;
            this.speedScalar = speedScalar;
        }
    }
}