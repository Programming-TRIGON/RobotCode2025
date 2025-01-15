package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.grapple.lasercan.LaserCAN;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Gripper extends MotorSubsystem {
    private final TalonFXMotor grippingMotor = GripperConstants.GRIPPING_MOTOR;
    private final TalonFXMotor angleMotor = GripperConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = GripperConstants.ANGLE_ENCODER;
    private final LaserCAN laserCAN = GripperConstants.LASER_CAN;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(GripperConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(GripperConstants.FOC_ENABLED);
    private GripperConstants.GripperState targetState = GripperConstants.GripperState.REST;

    public Gripper() {
        setName("Gripper");
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
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
        grippingMotor.update();
        angleMotor.update();
        angleEncoder.update();
        laserCAN.update();
    }

    @Override
    public void stop() {
        grippingMotor.stopMotor();
        angleMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("GripperAngleMotor")
                .angularPosition(Units.Rotations.of(angleEncoder.getSignal(CANcoderSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        Logger.recordOutput("GripperPose", calculateVisualizationPose());

        GripperConstants.GRIPPING_MECHANISM.update(
                grippingMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)
        );
        GripperConstants.ANGLE_MECHANISM.update(
                getCurrentEncoderAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    public boolean hasGamePiece() {
        return laserCAN.hasResult() && laserCAN.getDistanceMillimeters() < GripperConstants.GAME_PIECE_DETECTION_THRESHOLD_MILLIMETERS;
    }

    void setTargetState(GripperConstants.GripperState targetState) {
        this.targetState = targetState;

        setTargetState(
                targetState.targetAngle,
                targetState.targetGripperVoltage
        );
    }

    void setTargetState(Rotation2d targetAngle, double targetGrippingVoltage) {
        setTargetAngle(targetAngle);
        setTargetVoltage(targetGrippingVoltage);
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private void setTargetVoltage(double targetGrippingVoltage) {
        GripperConstants.GRIPPING_MECHANISM.setTargetVelocity(targetGrippingVoltage);
        grippingMotor.setControl(voltageRequest.withOutput(targetGrippingVoltage));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d originPoint = GripperConstants.GRIPPER_VISUALIZATION_ORIGIN_POINT;
        final Pose3d elevatorPose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

        final Transform3d gripperToElevatorOrigin = new Transform3d(
                originPoint.relativeTo(elevatorPose).getTranslation(),
                originPoint.relativeTo(elevatorPose).getRotation()
        );
        return originPoint.transformBy(gripperToElevatorOrigin);
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }
}