package frc.trigon.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.PathPlannerConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.simulation.SimpleMotorSimulation;

public class SwerveModuleConstants {
    static final boolean ENABLE_FOC = true;

    public static final double MAXIMUM_MODULE_ROTATIONAL_SPEED_RADIANS_PER_SECOND = edu.wpi.first.math.util.Units.rotationsToRadians(7); //TODO: calibrate
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    public static final SysIdRoutine.Config DRIVE_MOTOR_SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(5),
            Units.Second.of(1000)
    );

    /**
     * Creates a new SimpleMotorSimulation for the drive motor.
     * We use a function instead of a constant because we need to create a new instance of the simulation for each module.
     *
     * @return the drive motor simulation
     */
    static SimpleMotorSimulation createDriveMotorSimulation() {
        return new SimpleMotorSimulation(DCMotor.getKrakenX60Foc(1), 5.36, 0.003);
    }

    /**
     * Creates a new SimpleMotorSimulation for the steer motor.
     * We use a function instead of a constant because we need to create a new instance of the simulation for each module.
     *
     * @return the steer motor simulation
     */
    static SimpleMotorSimulation createSteerMotorSimulation(boolean isFront) {
        return new SimpleMotorSimulation(DCMotor.getFalcon500Foc(1), isFront ? 18.75 : 12.8, 0.003);
    }

    static TalonFXConfiguration generateDriveMotorConfiguration() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = 5.36;

        final double driveMotorSlipCurrent = PathPlannerConstants.ROBOT_CONFIG.moduleConfig.driveCurrentLimit;
        config.TorqueCurrent.PeakForwardTorqueCurrent = driveMotorSlipCurrent;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -driveMotorSlipCurrent;
        config.CurrentLimits.StatorCurrentLimit = driveMotorSlipCurrent;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 50 : 22;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.4708 : 6.2;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.48818 : 2.6;

        config.Feedback.VelocityFilterTimeConstant = 0;

        return config;
    }

    static TalonFXConfiguration generateSteerMotorConfiguration(boolean isFront, int feedbackRemoteSensorID) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = isFront ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit = RobotHardwareStats.isSimulation() ? 200 : 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Feedback.RotorToSensorRatio = isFront ? 18.75 : 12.8;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = feedbackRemoteSensorID;

        config.Slot0.kP = isFront ?
                (RobotHardwareStats.isSimulation() ? 120 : 55) :
                (RobotHardwareStats.isSimulation() ? 120 : 55);
        config.Slot0.kI = isFront ?
                (RobotHardwareStats.isSimulation() ? 0 : 0) :
                (RobotHardwareStats.isSimulation() ? 0 : 0);
        config.Slot0.kD = isFront ?
                (RobotHardwareStats.isSimulation() ? 0 : 0) :
                (RobotHardwareStats.isSimulation() ? 0 : 0);
        config.ClosedLoopGeneral.ContinuousWrap = true;

        return config;
    }

    static CANcoderConfiguration generateSteerEncoderConfiguration(double offsetRotations) {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = offsetRotations;

        return config;
    }
}