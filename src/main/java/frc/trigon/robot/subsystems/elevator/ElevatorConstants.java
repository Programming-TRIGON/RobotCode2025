package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.ElevatorSimulation;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;

public class ElevatorConstants {
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(12, "ElevatorMasterMotor"),
            FOLLOWER_MOTOR = new TalonFXMotor(13, "ElevatorFollowerMotor");

    static final boolean FOC_ENABLED = true;

    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            DCMotor.getFalcon500Foc(2),
            7.222222,
            8,
            0.025,
            0.73,//AFTER SEASON TODO: remove the need for this
            1.8,
            true
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.25).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    public static final Pose3d FIRST_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(-0.120, 0, 0.131),
            new Rotation3d(0, 0, 0)
    );
    static final Pose3d SECOND_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(-0.120, 0, 0.1111),
            new Rotation3d(0, 0, 0)
    );


    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            1.8 + 0.1,
            0.73,
            Color.kYellow
    );

    static final double FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS = 0.6;
    static final double DRUM_DIAMETER_METERS = 0.05;
    static final double POSITION_TOLERANCE_METERS = 0.02;
    static final double
            GRIPPER_HITTING_ELEVATOR_BASE_LOWER_BOUND_POSITION_ROTATIONS = 0.1,
            GRIPPER_HITTING_ELEVATOR_BASE_UPPER_BOUND_POSITION_ROTATIONS = 0.4;

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 7.222222;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 40 : 0.6;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.22774 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.066659 : 0.079427;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.74502 : 0.88;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.30539 : 0.42;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.6;

        config.MotionMagic.MotionMagicCruiseVelocity = RobotHardwareStats.isSimulation() ? 80 : 10;
        config.MotionMagic.MotionMagicAcceleration = RobotHardwareStats.isSimulation() ? 80 : 50;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);

        MASTER_MOTOR.setPosition(0);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR.getID(), true);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    public enum ElevatorState {
        REST(0),
        SCORE_L1(0),
        SCORE_L2(0),
        SCORE_L3(0.43),
        SCORE_L4(1.02),
        COLLECT_ALGAE_FROM_L3(0.365),
        SCORE_NET(1.04);

        public final double targetPositionMeters;

        ElevatorState(double targetPositionMeters) {
            this.targetPositionMeters = targetPositionMeters;
        }
    }
}