package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.grapple.lasercan.LaserCAN;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Gripper extends MotorSubsystem {
    private final TalonFXMotor gripperMotor = GripperConstants.GRIPPER_MOTOR;
    private final TalonFXMotor angleMotor = GripperConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = GripperConstants.ANGLE_ENCODER;
    private final LaserCAN laserCAN = GripperConstants.LASER_CAN;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(GripperConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(GripperConstants.FOC_ENABLED);
    private GripperConstants.GripperState targetState = GripperConstants.GripperState.STOP;

    public Gripper() {
        setName("Gripper");
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return GripperConstants.SYSID_CONFIG;
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updatePeriodically() {
        gripperMotor.update();
        angleMotor.update();
        angleEncoder.update();
        laserCAN.update();
    }

    @Override
    public void stop() {
        gripperMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("AngleEncoder")
                .linearPosition(Units.Meters.of(angleMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        GripperConstants.GRIPPER_MECHANISM.update(gripperMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE), targetState.targetGripperVoltage);
        GripperConstants.ARM_MECHANISM.update(getCurrentEncoderAngle(), targetState.targetAngle);
    }

    void stopMotors() {
        gripperMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public boolean hasGamePiece() {
        return laserCAN.getDistanceMillimeters() < GripperConstants.GAME_PEICE_DETECTION_THRESHOLD;
    }

    void setTargetState(GripperConstants.GripperState targetState) {
        this.targetState = targetState;

        setTargetState(
                targetState.targetGripperVoltage,
                targetState.targetAngle
        );
    }

    void setTargetState(double targetVoltage, Rotation2d targetAngle) {
        setTargetVoltage(targetVoltage);
        setTargetAngle(targetAngle);
    }

    void setTargetVoltage(double targetVoltage) {
        gripperMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }
}