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
            angleMotor = CoralIntakeConstants.MASTER_ANGLE_MOTOR;
    private final CANcoderEncoder encoder = CoralIntakeConstants.ANGLE_ENCODER;
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
        CoralIntakeConstants.INTAKE_MECHANISM.update(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE), targetState.targetIntakeVoltage);
        CoralIntakeConstants.FUNNEL_MECHANISM.update(funnelMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE), targetState.targetFunnelVoltage);
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
        encoder.update();
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

    public boolean atTargetState() {
        return Math.abs(getCurrentEncoderAngle().minus(targetState.targetAngle).getDegrees()) < CoralIntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public boolean atState(CoralIntakeConstants.CoralIntakeState targetState) {
        return targetState == this.targetState && atTargetState();
    }

    public boolean hasGamePiece() {
        return CoralIntakeConstants.CORAL_COLLECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    /**
     * Checks if a coral has been collected early using the motor's current.
     * This is quicker than {@linkplain CoralIntake#hasGamePiece()} since it updates from the change in current (which happens right when we hit the coral),
     * instead of the distance sensor which is positioned later on the system.
     *
     * @return whether an early coral collection has been detected
     */
    public boolean isEarlyCoralCollectionDetected() {
        return CoralIntakeConstants.EARLY_NOTE_COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
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

    void setTargetIntakeVoltage(double targetVoltage) {
        intakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    void setTargetFunnelVoltage(double targetVoltage) {
        funnelMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d originPoint = CoralIntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT;
        final Transform3d intakeTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, getCurrentEncoderAngle().getRadians(), 0)
        );

        return originPoint.transformBy(intakeTransform);
    }
}