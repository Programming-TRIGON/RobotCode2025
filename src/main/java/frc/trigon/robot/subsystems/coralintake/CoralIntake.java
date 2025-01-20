package frc.trigon.robot.subsystems.coralintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class CoralIntake extends MotorSubsystem {
    private final TalonFXMotor
            intakeMotor = CoralIntakeConstants.INTAKE_MOTOR,
            funnelMotor = CoralIntakeConstants.FUNNEL_MOTOR,
            angleMotor = CoralIntakeConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = CoralIntakeConstants.ANGLE_ENCODER;
    private final SimpleSensor beamBreak = CoralIntakeConstants.BEAM_BREAK;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(CoralIntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(CoralIntakeConstants.FOC_ENABLED);
    private CoralIntakeConstants.CoralIntakeState targetState = CoralIntakeConstants.CoralIntakeState.REST;

    public CoralIntake() {
        setName("CoralIntake");
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("CoralAngleMotor")
                .angularPosition(Units.Rotations.of(getCurrentEncoderAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        CoralIntakeConstants.INTAKE_MECHANISM.update(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        CoralIntakeConstants.FUNNEL_MECHANISM.update(funnelMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        CoralIntakeConstants.ANGLE_MECHANISM.update(
                getCurrentEncoderAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        Logger.recordOutput("Poses/Components/CoralIntakePose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        intakeMotor.update();
        funnelMotor.update();
        angleMotor.update();
        angleEncoder.update();
        beamBreak.updateSensor();
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return CoralIntakeConstants.ANGLE_SYSID_CONFIG;
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
        funnelMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public boolean atState(CoralIntakeConstants.CoralIntakeState targetState) {
        return targetState == this.targetState && atTargetAngle();
    }

    public boolean atTargetAngle() {
        final double angleDifferenceFromTargetAngleDegrees = Math.abs(getCurrentEncoderAngle().minus(targetState.targetAngle).getDegrees());
        return angleDifferenceFromTargetAngleDegrees < CoralIntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public boolean hasGamePiece() {
        return beamBreak.getBinaryValue();
    }

    void setTargetState(CoralIntakeConstants.CoralIntakeState targetState) {
        this.targetState = targetState;

        setTargetState(
                targetState.targetIntakeVoltage,
                targetState.targetFunnelVoltage,
                targetState.targetAngle
        );
    }

    void setTargetState(double targetIntakeVoltage, double targetFunnelVoltage, Rotation2d targetAngle) {
        setTargetVoltage(targetIntakeVoltage, targetFunnelVoltage);
        setTargetAngle(targetAngle);
    }

    void setTargetVoltage(double targetIntakeVoltage, double targetFunnelVoltage) {
        setTargetIntakeVoltage(targetIntakeVoltage);
        setTargetFunnelVoltage(targetFunnelVoltage);
    }

    private void setTargetIntakeVoltage(double targetVoltage) {
        CoralIntakeConstants.INTAKE_MECHANISM.setTargetVelocity(targetVoltage);
        intakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private void setTargetFunnelVoltage(double targetVoltage) {
        CoralIntakeConstants.FUNNEL_MECHANISM.setTargetVelocity(targetVoltage);
        funnelMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d originPoint = CoralIntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT;

        final Transform3d intakeTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, getCurrentEncoderAngle().getRadians(), 0)
        );

        return originPoint.transformBy(intakeTransform);
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }
}