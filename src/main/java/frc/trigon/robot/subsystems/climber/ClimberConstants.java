package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.climber.climbervisualization.ClimberVisualization;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;

public class ClimberConstants {
    private static final int MOTOR_ID = 16;
    private static final String MOTOR_NAME = "ClimberMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final boolean ENABLE_FOC = true;
    private static final double
            GROUNDED_P = RobotHardwareStats.isSimulation() ? 100 : 0,
            GROUNDED_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_KS = RobotHardwareStats.isSimulation() ? 0.016805 : 0,
            GROUNDED_KV = RobotHardwareStats.isSimulation() ? 4.0291 : 0,
            GROUNDED_KA = RobotHardwareStats.isSimulation() ? 0.014829 : 0;
    private static final double
            ON_CAGE_P = RobotHardwareStats.isSimulation() ? GROUNDED_P : 0,
            ON_CAGE_I = RobotHardwareStats.isSimulation() ? GROUNDED_I : 0,
            ON_CAGE_D = RobotHardwareStats.isSimulation() ? GROUNDED_D : 0,
            ON_CAGE_KS = RobotHardwareStats.isSimulation() ? GROUNDED_KS : 0,
            ON_CAGE_KV = RobotHardwareStats.isSimulation() ? GROUNDED_KV : 0,
            ON_CAGE_KA = RobotHardwareStats.isSimulation() ? GROUNDED_KA : 0;
    static final double ON_CAGE_KG = 0;
    static final double
            MAXIMUM_GROUNDED_VELOCITY = 100,
            MAXIMUM_GROUNDED_ACCELERATION = 100,
            MAXIMUM_ON_CAGE_VELOCITY = 100,
            MAXIMUM_ON_CAGE_ACCELERATION = 100;
    static final int
            GROUNDED_PID_SLOT = 0,
            ON_CAGE_PID_SLOT = 1;
    private static final double FORWARD_SOFT_LIMIT_POSITION_ROTATIONS = 3;
    private static final double REVERSE_LIMIT_SWITCH_PRESSED_POSITION_ROTATIONS = 0;
    private static final ReverseLimitTypeValue REVERSE_LIMIT_TYPE = ReverseLimitTypeValue.NormallyOpen;
    static final double GEAR_RATIO = 33.75;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation MOTOR_SIMULATION = new SimpleMotorSimulation(GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    static final ClimberVisualization CLIMBER_VISUALIZATION = new ClimberVisualization();

    static final double CLIMBER_TOLERANCE_METERS = 0.01;
    public static final double
            MANUALLY_RAISE_CLIMBER_VOLTAGE = 2,
            MANUALLY_LOWER_CLIMBER_VOLTAGE = -2;

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

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT_POSITION_ROTATIONS;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = REVERSE_LIMIT_SWITCH_PRESSED_POSITION_ROTATIONS;
        config.HardwareLimitSwitch.ReverseLimitType = REVERSE_LIMIT_TYPE;

        config.MotionMagic.MotionMagicCruiseVelocity = MAXIMUM_GROUNDED_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_GROUNDED_ACCELERATION;

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