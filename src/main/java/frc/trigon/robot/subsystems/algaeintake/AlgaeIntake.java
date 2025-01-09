package frc.trigon.robot.subsystems.algaeintake;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    private final CANcoderEncoder encoder = AlgaeIntakeConstants.ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(AlgaeIntakeConstants.ENABLE_FOC);
    private final MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
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
    public void drive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("AlgaeAngleMotor")
                .angularPosition(Units.Rotations.of(getEncoderPosition().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(encoder.getSignal(CANcoderSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        AlgaeIntakeConstants.INTAKE_MECHANISM.update(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        AlgaeIntakeConstants.ANGLE_MECHANISM.update(
                getEncoderPosition(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    @Override
    public void updatePeriodically() {
        intakeMotor.update();
        angleMotor.update();
        encoder.update();

        Logger.recordOutput("Poses/Components/AlgaeIntake", calculateVisualizationPose());
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return AlgaeIntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        stopIntakeMotor();
        stopAngleMotor();
    }

    public boolean atTargetAngle() {
        return Math.abs(getEncoderPosition().minus(targetState.targetAngle).getDegrees()) < AlgaeIntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    void stopIntakeMotor() {
        intakeMotor.stopMotor();
    }

    void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    void setTargetState(AlgaeIntakeConstants.AlgaeIntakeState targetState) {
        this.targetState = targetState;
        setTargetVoltage(targetState.targetVoltage);
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    void setTargetVoltage(double targetVoltage) {
        intakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
        AlgaeIntakeConstants.INTAKE_MECHANISM.setTargetVelocity(targetVoltage);
    }

    Rotation2d getEncoderPosition() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }

    Rotation2d getRotorPosition() {
        return Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.ROTOR_POSITION));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d originPoint = AlgaeIntakeConstants.ALGAE_INTAKE_VISUALIZATION_ORIGIN_POINT;
        return new Pose3d(
                originPoint.getTranslation(),
                new Rotation3d(originPoint.getRotation().getX(), originPoint.getRotation().getY() + getEncoderPosition().getRadians(), originPoint.getRotation().getZ())
        );
    }
}