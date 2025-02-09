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
    
    static final boolean ENABLE_FOC = true;
    static final double ON_CAGE_KG = 0;
    static final double
            MAXIMUM_GROUNDED_VELOCITY = 100,
            MAXIMUM_GROUNDED_ACCELERATION = 100,
            MAXIMUM_ON_CAGE_VELOCITY = 100,
            MAXIMUM_ON_CAGE_ACCELERATION = 100;
    static final int
            GROUNDED_PID_SLOT = 0,
            ON_CAGE_PID_SLOT = 1;
    private static final double GEAR_RATIO = 33.75;

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

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 100 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.016805 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 4.0291 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.014829 : 0;

        config.Slot1.kP = RobotHardwareStats.isSimulation() ? config.Slot0.kP : 0;
        config.Slot1.kI = RobotHardwareStats.isSimulation() ? config.Slot0.kI : 0;
        config.Slot1.kD = RobotHardwareStats.isSimulation() ? config.Slot0.kD : 0;
        config.Slot1.kS = RobotHardwareStats.isSimulation() ? config.Slot0.kS : 0;
        config.Slot1.kV = RobotHardwareStats.isSimulation() ? config.Slot0.kV : 0;
        config.Slot1.kA = RobotHardwareStats.isSimulation() ? config.Slot0.kA : 0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 3;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

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