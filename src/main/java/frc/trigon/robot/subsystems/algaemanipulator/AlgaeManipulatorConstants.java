package frc.trigon.robot.subsystems.algaemanipulator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;

public class AlgaeManipulatorConstants {
    private static final int ANGLE_MOTOR_ID = 16;
    private static final String ANGLE_MOTOR_NAME = "AlgaeManipulatorAngleMotor";
    static final TalonFXMotor ANGLE_MOTOR = new TalonFXMotor(ANGLE_MOTOR_ID, ANGLE_MOTOR_NAME);

    private static final double GEAR_RATIO = 24.375;
    static final boolean FOC_ENABLED = true;

    private static final int ANGLE_MOTOR_AMOUNT = 1;
    private static final DCMotor ANGLE_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(ANGLE_MOTOR_AMOUNT);
    private static final double
            ARM_LENGTH_METERS = 0.35, //TODO: Measure this
            ARM_MASS_KILOGRAMS = 3;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation ANGLE_MOTOR_SIMULATION = new SingleJointedArmSimulation(
            ANGLE_MOTOR_GEARBOX,
            GEAR_RATIO,
            ARM_LENGTH_METERS,
            ARM_MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    static final SingleJointedArmMechanism2d ANGLE_MECHANISM = new SingleJointedArmMechanism2d(
            "AlgaeManipulatorAngleMechanism",
            ARM_LENGTH_METERS,
            Color.kRed
    );

    public static final double MAXIMUM_CURRENT_REQUEST_DUTY_CYCLE = 0.5;
    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1);
    static final Rotation2d MAXIMUM_RESTING_GRIPPER_ANGLE = Rotation2d.fromDegrees(0);
    static final double
            OPEN_TORQUE_CURRENT = 15,
            CLOSE_TO_LIMIT_TORQUE_CURRENT = -10;

    static {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 100 : 85;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0.1;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 100 : 22;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 100 : 18;
        config.MotionMagic.MotionMagicJerk = 0;

        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;
        config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = GripperConstants.GRIPPING_MOTOR_ID;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

//        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_ANGLE.getRotations();

        ANGLE_MOTOR.applyConfiguration(config);
        ANGLE_MOTOR.setPhysicsSimulation(ANGLE_MOTOR_SIMULATION);

        ANGLE_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.REVERSE_LIMIT, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        ANGLE_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
    }

    public enum AlgaeManipulatorState {
        REST(Rotation2d.fromDegrees(10)),
        OPEN_FOR_GRIPPER(Rotation2d.fromDegrees(30));

        final Rotation2d targetAngle;

        AlgaeManipulatorState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }
    }
}