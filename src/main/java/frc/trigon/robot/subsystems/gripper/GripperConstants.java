package frc.trigon.robot.subsystems.gripper;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import org.trigon.hardware.grapple.lasercan.LaserCAN;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;


public class GripperConstants {
    private static final int
            MOTOR_ID = 15,
            LASER_CAN_ID = 15;
    private static final String
            MOTOR_NAME = "GrippperMotor",
            LASER_CAN_NAME = "GripperLaserCAN";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);
    static final LaserCAN LASER_CAN = new LaserCAN(LASER_CAN_ID, LASER_CAN_NAME);

    private static final double GEAR_RATIO = 1;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final int
    LASER_CAN_X_AXIS_INTEREST = 6,
    LASER_CAN_Y_AXIS_INTEREST = 6,
    LASER_CAN_WIDTH_INTEREST = 4,
    LASER_CAN_HEIGHT_INTEREST = 4;
    private static final LaserCan.RangingMode LASER_CAN_RANGING_MODE = LaserCan.RangingMode.SHORT;
    private static final LaserCan.TimingBudget LASER_CAN_LOOP_TIME = LaserCan.TimingBudget.TIMING_BUDGET_33MS;
    private static final DoubleSupplier LASER_CAN_SIMULATION_SUPPLIER = ()-> {
        double DISTANCE = LASER_CAN.getDistanceMillimeters();
        double DISTANCE_THRESHOLD = 100.0;
        if (DISTANCE < DISTANCE_THRESHOLD) {
            return 1;
        } else  {
            return 0;
        }
    };
    static final boolean FOC_ENABLED = true;

    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    private static final int NUMBER_OF_MOTORS = 1;

    private static final DCMotor GEARBOX = DCMotor.getKrakenX60(NUMBER_OF_MOTORS);

    static final SpeedMechanism2d GRIPPER_MECHANISM = new SpeedMechanism2d("GripperMechanism", MAXIMUM_DISPLAYABLE_VELOCITY);
    private static final SimpleMotorSimulation GRIPPER_SIMULATION = new SimpleMotorSimulation(GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA);

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

        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    private static void configureLaser() {
        try {
            LASER_CAN.setRegionOfInterest(LASER_CAN_X_AXIS_INTEREST, LASER_CAN_Y_AXIS_INTEREST, LASER_CAN_WIDTH_INTEREST, LASER_CAN_HEIGHT_INTEREST);
            LASER_CAN.setRangingMode(LASER_CAN_RANGING_MODE);
            LASER_CAN.setLoopTime(LASER_CAN_LOOP_TIME);
            LASER_CAN.setSimulationSupplier(LASER_CAN_SIMULATION_SUPPLIER);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
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