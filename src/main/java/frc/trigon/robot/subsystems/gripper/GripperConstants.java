package frc.trigon.robot.subsystems.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.interfaces.LaserCanInterface;
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
import org.trigon.hardware.grapple.lasercan.LaserCAN;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;


public class GripperConstants {
    static final TalonFXMotor
            GRIPPING_MOTOR = new TalonFXMotor(14, "GripperGrippingMotor"),
            ANGLE_MOTOR = new TalonFXMotor(15, "GripperAngleMotor");
    static final CANcoderEncoder ANGLE_ENCODER = new CANcoderEncoder(15, "GripperAngleEncoder");
    static final LaserCAN LASER_CAN = new LaserCAN(14, "GripperLaserCAN");

    static final double ANGLE_MOTOR_GEAR_RATIO = 34.642351;

    static final double POSITION_OFFSET_FROM_GRAVITY_OFFSET = 0.04379;

    static final boolean FOC_ENABLED = true;

    private static final SimpleMotorSimulation GRIPPING_SIMULATION = new SimpleMotorSimulation(
            DCMotor.getFalcon500Foc(1),
            4,
            0.003
    );
    private static final SingleJointedArmSimulation ARM_SIMULATION = new SingleJointedArmSimulation(
            DCMotor.getKrakenX60Foc(1),
            ANGLE_MOTOR_GEAR_RATIO,
            0.24,
            3,
            Rotation2d.fromDegrees(-55),
            Rotation2d.fromDegrees(120),
            true
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Seconds),
            Units.Volts.of(1),
            Units.Second.of(1000)
    );

    static final SpeedMechanism2d GRIPPING_MECHANISM = new SpeedMechanism2d(
            "GripperGrippingMechanism",
            12
    );
    static final SingleJointedArmMechanism2d ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            "GripperAngleMechanism",
            0.24,
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
    static final BooleanEvent COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> LASER_CAN.hasResult() && LASER_CAN.getDistanceMillimeters() < 10
    ).debounce(0.14);

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

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.RotorToSensorRatio = 4;

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

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 100 : 8;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0.1579;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 4.0791;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0 : 0.23721;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 5 : 3;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 5 : 8;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Rotation2d.fromDegrees(-59.645756).minus(Rotation2d.fromRotations(POSITION_OFFSET_FROM_GRAVITY_OFFSET)).getRotations();

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotation2d.fromDegrees(120).minus(Rotation2d.fromRotations(POSITION_OFFSET_FROM_GRAVITY_OFFSET)).getRotations();

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

        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -0.36379;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ANGLE_ENCODER.applyConfiguration(config);
        ANGLE_ENCODER.setSimulationInputsFromTalonFX(ANGLE_MOTOR);

        ANGLE_ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ANGLE_ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void configureLaserCAN() {
        try {
            LASER_CAN.setRegionOfInterest(6, 6, 12, 12);
            LASER_CAN.setRangingMode(LaserCanInterface.RangingMode.SHORT);
            LASER_CAN.setLoopTime(LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS);
            LASER_CAN.setSimulationSupplier(
                    () -> SimulationFieldHandler.isCoralInGripper() &&
                            SimulationFieldHandler.isHoldingCoral() ? 1 : 1000
            );
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }

    public enum GripperState {
        REST(Rotation2d.fromDegrees(-50), 0),
        EJECT(Rotation2d.fromDegrees(55), -3),
        SCORE_L4(Rotation2d.fromDegrees(55), -8),
        SCORE_L3_OR_L2(Rotation2d.fromDegrees(55), SCORE_L4.targetGripperVoltage),
        SCORE_L1(Rotation2d.fromDegrees(93), SCORE_L4.targetGripperVoltage),
        LOAD_CORAL(Rotation2d.fromDegrees(-50), 11),
        UNLOAD_CORAL(Rotation2d.fromDegrees(-50), -3),
        OPEN_FOR_ELEVATOR_MINIMUM(Rotation2d.fromDegrees(-33), 0),
        COLLECT_ALGAE_FROM_REEF(Rotation2d.fromDegrees(32), -35),
        SCORE_ALGAE_IN_NET(Rotation2d.fromDegrees(110), 11),
        PREPARE_FOR_SCORING_ALGAE_IN_NET(SCORE_ALGAE_IN_NET.targetAngle, COLLECT_ALGAE_FROM_REEF.targetGripperVoltage),
        AFTER_ELEVATOR_OPEN_POSITION(Rotation2d.fromDegrees(0), 0),
        COLLECT_CORAL_FROM_FEEDER(Rotation2d.fromDegrees(140), -6); //TODO: Calibrate

        final Rotation2d targetAngle;
        final double targetGripperVoltage;

        GripperState(Rotation2d targetAngle, double targetVoltage) {
            this.targetAngle = targetAngle;
            this.targetGripperVoltage = targetVoltage;
        }
    }
}