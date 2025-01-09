package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.ElevatorSimulation;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;

public class ElevatorConstants {
    private static final int
            MASTER_MOTOR_ID = 12,
            FOLLOWER_MOTOR_ID = 13,
            ENCODER_ID = 12;
    private static final String
            MASTER_MOTOR_NAME = "ElevatorMasterMotor",
            FOLLOWER_MOTOR_NAME = "ElevatorFollowerMotor",
            ENCODER_NAME = "ElevatorEncoder";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME, RobotConstants.CANIVORE_NAME);

    private static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE = false;
    private static final double ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 1;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final double ENCODER_MAGNET_OFFSET_VALUE = 0;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.RemoteCANcoder;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KG = 0,
            KA = 0;
    private static final double GEAR_RATIO = 1;
    static final double
            MOTION_MAGIC_CRUISE_VELOCITY = 25,
            MOTION_MAGIC_ACCELERATION = 25;
    static final boolean FOC_ENABLED = true;

    private static final double
            MASS_KILOGRAMS = 5,
            DRUM_RADIUS_METERS = 0.5,
            MAXIMUM_HEIGHT_METERS = 1,
            RETRACTED_ELEVATOR_LENGTH_METERS = 0.5;
    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(GEARBOX, GEAR_RATIO, MASS_KILOGRAMS, DRUM_RADIUS_METERS, RETRACTED_ELEVATOR_LENGTH_METERS, MAXIMUM_HEIGHT_METERS, true);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Seconds.of(1).unit()),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );
    static final Pose3d ELEVATOR_ORIGIN_POINT = new Pose3d(1, 0, 1, new Rotation3d(edu.wpi.first.math.util.Units.degreesToRadians(10), 0, 0));
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            MAXIMUM_HEIGHT_METERS,
            RETRACTED_ELEVATOR_LENGTH_METERS,
            Color.kYellow
    );
    static final double DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;

    static {
        configureMasterMotor();
        configureFollowerMotor();
        configureEncoder();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = MOTOR_NEUTRAL_MODE;
        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = MOTOR_NEUTRAL_MODE;
        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;

        FOLLOWER_MOTOR.applyConfiguration(config);
        FOLLOWER_MOTOR.setControl(new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE));
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ABSOLUTE_SENSOR_DISCONTINUITY_POINT;
        config.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET_VALUE;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MASTER_MOTOR);
    }

    public enum ElevatorState {
        RESTING(0, 100),
        REEF_L1(0.1, 100),
        REEF_L2(0.2, 10),
        REEF_L3(0.3, 10),
        REEF_L4(0.4, 10);

        final double positionMeters;
        final double speedPercentage;

        ElevatorState(double positionMeters, double speedPercentage) {
            this.positionMeters = positionMeters;
            this.speedPercentage = speedPercentage;
        }
    }
}