package frc.trigon.robot.subsystems.algaeintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class AlgaeIntake extends MotorSubsystem {
    private final TalonFXMotor
            intakeMotor = AlgaeIntakeConstants.INTAKE_MOTOR,
            angleMotor = AlgaeIntakeConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = AlgaeIntakeConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(AlgaeIntakeConstants.ENABLE_FOC);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(AlgaeIntakeConstants.ENABLE_FOC);
    private AlgaeIntakeConstants.AlgaeIntakeState targetState = AlgaeIntakeConstants.AlgaeIntakeState.REST;

    public AlgaeIntake() {
        setName("AlgaeIntake");
    }

    @Override
    public void setBrake(boolean brake) {
        intakeMotor.setBrake(brake);
        angleMotor.setBrake(brake);
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("AlgaeAngleMotor")
                .angularPosition(Units.Rotations.of(getAngleEncoderPosition().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        AlgaeIntakeConstants.INTAKE_MECHANISM.update(
                intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE),
                targetState.targetVoltage
        );
        AlgaeIntakeConstants.ANGLE_MECHANISM.update(
                getAngleEncoderPosition(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        Logger.recordOutput("Poses/Components/AlgaeIntake", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        intakeMotor.update();
        angleMotor.update();
        angleEncoder.update();
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return AlgaeIntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public boolean atState(AlgaeIntakeConstants.AlgaeIntakeState targetState) {
        return targetState == this.targetState && atTargetState();
    }

    public boolean atTargetState() {
        return Math.abs(getAngleEncoderPosition().minus(targetState.targetAngle).getDegrees()) < AlgaeIntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    void setTargetState(AlgaeIntakeConstants.AlgaeIntakeState targetState) {
        this.targetState = targetState;
        setTargetState(targetState.targetVoltage, targetState.targetAngle);
    }

    void setTargetState(double targetVoltage, Rotation2d targetAngle) {
        setTargetVoltage(targetVoltage);
        setTargetAngle(targetAngle);
    }

    private void setTargetVoltage(double targetVoltage) {
        intakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d originPoint = AlgaeIntakeConstants.ALGAE_INTAKE_VISUALIZATION_ORIGIN_POINT;
        final Transform3d transform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, getAngleEncoderPosition().getRadians(), 0)
        );
        return originPoint.transformBy(transform);
    }
    
    private Rotation2d getAngleEncoderPosition() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }
}