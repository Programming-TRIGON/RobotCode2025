package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;

public class ClimberConstants {
    private static final int MOTOR_ID = 15;
    private static final String MOTOR_NAME = "ClimberMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final boolean ENABLE_FOC = true;
    private static final double
            GROUNDED_P = 1,
            GROUNDED_I = 0,
            GROUNDED_D = 0,
            GROUNDED_KS = 0,
            GROUNDED_KV = 0,
            GROUNDED_KA = 0;
    private static final double
            ON_CAGE_P = 1,
            ON_CAGE_I = 0,
            ON_CAGE_D = 0,
            ON_CAGE_KS = 0,
            ON_CAGE_KV = 0,
            ON_CAGE_KA = 0;
    static final double ON_CAGE_KG = 0;
    static final double
            MAX_GROUNDED_VELOCITY = 100,
            MAX_GROUNDED_ACCELERATION = 100,
            MAX_ON_CAGE_VELOCITY = 100,
            MAX_ON_CAGE_ACCELERATION = 100;
    static final int
            GROUNDED_SLOT = 0,
            ON_CAGE_SLOT = 1;
    private static final double FORWARD_SOFT_LIMIT_POSITION_ROTATIONS = 0;
    private static final double REVERSE_LIMIT_SWITCH_RESET_POSITION = 0;
    private static final ReverseLimitTypeValue REVERSE_LIMIT_TYPE = ReverseLimitTypeValue.NormallyOpen;
    static final double GEAR_RATIO = 68.57;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation MOTOR_SIMULATION = new SimpleMotorSimulation(GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Seconds),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );

    static final Translation3d CLIMBER_ORIGIN_POINT = new Translation3d();
    static final SingleJointedArmMechanism2d CLIMBER_MECHANISM = new SingleJointedArmMechanism2d("ClimberMechanism", Color.kViolet);

    static final double CLIMBER_TOLERANCE_METERS = 0.01;

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = GROUNDED_P;
        config.Slot0.kI = GROUNDED_I;
        config.Slot0.kD = GROUNDED_D;
        config.Slot0.kS = GROUNDED_KS;
        config.Slot0.kV = GROUNDED_KV;
        config.Slot0.kA = GROUNDED_KA;

        config.Slot1.kP = ON_CAGE_P;
        config.Slot1.kI = ON_CAGE_I;
        config.Slot1.kD = ON_CAGE_D;
        config.Slot1.kS = ON_CAGE_KS;
        config.Slot1.kV = ON_CAGE_KV;
        config.Slot1.kA = ON_CAGE_KA;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FORWARD_SOFT_LIMIT_POSITION_ROTATIONS;

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = REVERSE_LIMIT_SWITCH_RESET_POSITION;
        config.HardwareLimitSwitch.ReverseLimitType = REVERSE_LIMIT_TYPE;

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_GROUNDED_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_GROUNDED_ACCELERATION;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(MOTOR_SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.REVERSE_LIMIT, 100);
    }

    public enum ClimberState {
        REST(0, false),
        PREPARE_FOR_CLIMB(2, false),
        CLIMB(0.1, true);

        public final double positionRotations;
        public final boolean isAffectedByRobotWeight;

        ClimberState(double positionRotations, boolean isAffectedByRobotWeight) {
            this.positionRotations = positionRotations;
            this.isAffectedByRobotWeight = isAffectedByRobotWeight;
        }
    }
}