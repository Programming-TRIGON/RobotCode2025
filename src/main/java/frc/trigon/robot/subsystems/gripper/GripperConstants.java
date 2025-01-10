package frc.trigon.robot.subsystems.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
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
            MOTOR_ID = 14,
            LASER_CAN_ID = 15;

    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, "GripperMotor");
    static final LaserCan LASER_CAN = new LaserCan(LASER_CAN_ID);

    private static final int
            NUMBER_OF_MOTORS = 1,
            X = 8,
            Y = 8,
            W = 16,
            H = 16;


    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final LaserCan.RangingMode LASER_CAN_RANGING_MODE = LaserCan.RangingMode.SHORT;
    private static final LaserCan.TimingBudget LASER_CAN_TIMING_BUDGET = LaserCan.TimingBudget.TIMING_BUDGET_33MS;
    private static final LaserCan.RegionOfInterest LASER_CAN_REGION_OF_INTEREST = new LaserCan.RegionOfInterest(X, Y, W, H);
    private static final LaserCan.Measurement LASER_CAN_MEASUREMENT = LASER_CAN.getMeasurement();

    private static final double
    P = 1,
    I = 1,
    D = 1,
    GEAR_RATIO = 0.5,
    MOMENT_OF_INERTIA = 0.003,
    MAXIMUM_DISPLAYABLE_VELOCITY = 12;

    static final boolean FOC_ENABLED = true;
    private static final DCMotor GEAR_BOX = DCMotor.getKrakenX60(NUMBER_OF_MOTORS);
    static final SpeedMechanism2d GRIPPER_MECHANISM = new SpeedMechanism2d("Gripper", MAXIMUM_DISPLAYABLE_VELOCITY);
    private static final SimpleMotorSimulation GRIPPER_SIMULATION = new SimpleMotorSimulation(GEAR_BOX, GEAR_RATIO, MOMENT_OF_INERTIA);

    static {
        configureMotor();
        configureLaser();
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

    private static void configureLaser() {
        try {
            LASER_CAN.setRangingMode(LASER_CAN_RANGING_MODE);
            LASER_CAN.setRegionOfInterest(LASER_CAN_REGION_OF_INTEREST);
            LASER_CAN.setTimingBudget(LASER_CAN_TIMING_BUDGET);
        } catch (ConfigurationFailedException e) {
            throw new RuntimeException(e);
        }

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