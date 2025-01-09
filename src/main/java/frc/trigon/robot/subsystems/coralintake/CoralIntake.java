package frc.trigon.robot.subsystems.coralintake;

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
    private final CANcoderEncoder encoder = CoralIntakeConstants.ENCODER;
    private final SimpleSensor beamBreak = CoralIntakeConstants.BEAM_BREAK;
    private final VoltageOut
            funnelVoltageRequest = new VoltageOut(0).withEnableFOC(CoralIntakeConstants.FOC_ENABLED);
    private final MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    private CoralIntakeConstants.CoralIntakeState targetState = CoralIntakeConstants.CoralIntakeState.REST;

    public CoralIntake() {
        setName("CoralIntake");
    }

    @Override
    public void setBrake(boolean brake) {
        intakeMotor.setBrake(brake);
        funnelMotor.setBrake(brake);
        angleMotor.setBrake(brake);
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        angleMotor.setControl(funnelVoltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("CoralAngleMotor")
                .angularPosition(Units.Rotations.of(getCurrentEncoderAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(encoder.getSignal(CANcoderSignal.VELOCITY)))
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
        return Math.abs(getCurrentEncoderAngle().minus(targetState.targetAngle).getDegrees()) < CoralIntakeConstants.ANGLE_TOLERANCE_DEGREES;
    }

    public boolean hasGamePiece() {
        return beamBreak.getBinaryValue();
    }

    void setTargetState(CoralIntakeConstants.CoralIntakeState targetState) {
        this.targetState = targetState;

        setTargetVoltage(targetState.intakeVoltage, targetState.funnelVoltage);
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetVoltage(double targetIntakeVoltage, double targetFunnelVoltage) {
        setTargetIntakeVoltage(targetIntakeVoltage);
        setTargetFunnelVoltage(targetFunnelVoltage);
    }

    void setTargetIntakeVoltage(double voltage) {
        intakeMotor.setControl(funnelVoltageRequest.withOutput(voltage));
    }

    void setTargetFunnelVoltage(double voltage) {
        funnelMotor.setControl(funnelVoltageRequest.withOutput(voltage));
    }

    void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(encoder.getSignal(CANcoderSignal.POSITION));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d intakeVisualizationOriginPoint = CoralIntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT;
        return new Pose3d(
                intakeVisualizationOriginPoint.getTranslation(),
                new Rotation3d(
                        intakeVisualizationOriginPoint.getRotation().getX(),
                        intakeVisualizationOriginPoint.getRotation().getY() + getCurrentEncoderAngle().getRadians(),
                        intakeVisualizationOriginPoint.getRotation().getZ()
                )
        );
    }
}