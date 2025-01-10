package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.ElevatorSimulation;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;

public class ElevatorConstants {
    private static final int
            MASTER_MOTOR_ID = 13,
            FOLLOWER_MOTOR_ID = 14;
    private static final String
            MASTER_MOTOR_NAME = "ElevatorMasterMotor",
            FOLLOWER_MOTOR_NAME = "ElevatorFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);

    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSES_MASTER = false;
    private static final double
            P = 110,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KG = 0,
            KA = 0;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Elevator_Static;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseClosedLoopSign;
    private static final Rotation2d FORWARD_HARD_LIMIT_ROTATIONS = Rotation2d.fromRotations(4);
    private static final Rotation2d REVERSE_HARD_LIMIT_ROTATIONS = Rotation2d.fromRotations(0);
    private static final double GEAR_RATIO = 5;
    static final double
            MOTION_MAGIC_CRUISE_VELOCITY = 25,
            MOTION_MAGIC_ACCELERATION = 25,
            MOTION_MAGIC_JERK = MOTION_MAGIC_ACCELERATION * 10;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double
            MASS_KILOGRAMS = 9,
            DRUM_RADIUS_METERS = 0.02,
            RETRACTED_ELEVATOR_LENGTH_METERS = 0.2,
            MAXIMUM_HEIGHT_METERS = 1.9,
            MAXIMUM_LENGTH_METERS = MAXIMUM_HEIGHT_METERS - 0.05;
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MASS_KILOGRAMS,
            DRUM_RADIUS_METERS,
            RETRACTED_ELEVATOR_LENGTH_METERS,
            MAXIMUM_HEIGHT_METERS,
            SHOULD_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Seconds),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );

    static final Pose3d ELEVATOR_ORIGIN_POINT = new Pose3d(1, 0, 1, new Rotation3d(edu.wpi.first.math.util.Units.degreesToRadians(10), 0, 0));
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            MAXIMUM_LENGTH_METERS,
            RETRACTED_ELEVATOR_LENGTH_METERS,
            Color.kYellow
    );

    static final double DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;

   static double targetStateMax(ElevatorState targetState) {
        return targetState.targetPositionRotations + 0.05;
    }

   static double TargetStateMin(ElevatorState targetState) {
        return targetState.targetPositionRotations - 0.05;
    }

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = REVERSE_HARD_LIMIT_ROTATIONS.getRotations();
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = FORWARD_HARD_LIMIT_ROTATIONS.getRotations();

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;

        FOLLOWER_MOTOR.applyConfiguration(config);
        final Follower followMasterMotor = new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSES_MASTER);
        FOLLOWER_MOTOR.setControl(followMasterMotor);
    }

    public enum ElevatorState {
        REST(0),
        REEF_L1(0.1),
        REEF_L2(0.2),
        REEF_L3(0.3),
        REEF_L4(0.4);

        final double targetPositionRotations;

        ElevatorState(double targetPositionRotations) {
            this.targetPositionRotations = targetPositionRotations;
        }
    }
}