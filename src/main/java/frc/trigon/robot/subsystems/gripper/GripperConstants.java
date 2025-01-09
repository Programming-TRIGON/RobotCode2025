package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

public class GripperConstants {
    private static final int
            MOTOR_ID = 14;

    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, "GripperMotor");

    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

    private static final double
            P = 1,
            I = 1,
            D = 1,
            GEAR_RATIO = 0.5,
            MOMENT_OF_INERTIA = 0.003,
            MAXIMUM_DISPLAYABLE_VELOCITY = 12;

    private static final int NUMBER_OF_MOTORS = 1;

    static final boolean FOC_ENABLED = true;
    private static final DCMotor GEAR_BOX = DCMotor.getKrakenX60(NUMBER_OF_MOTORS);
    static final SpeedMechanism2d GRIPPER_MECHANISM = new SpeedMechanism2d("Gripper", MAXIMUM_DISPLAYABLE_VELOCITY);
    private static final SimpleMotorSimulation GRIPPER_SIMULATION = new SimpleMotorSimulation(GEAR_BOX, GEAR_RATIO, MOMENT_OF_INERTIA);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        MOTOR.applyConfiguration(config);

        MOTOR.setPhysicsSimulation(GRIPPER_SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    public enum GripperState {
        STOP(0),
        RELEASE(-4),
        GRAB(4);

        final double targetVoltage;

        GripperState(double targetVoltage) {
            this.targetVoltage = targetVoltage;
        }
    }
}