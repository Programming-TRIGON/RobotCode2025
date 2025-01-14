package frc.trigon.robot.subsystems.coralintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class CoralIntake extends MotorSubsystem {
    private final TalonFXMotor
            intakeMotor = CoralIntakeConstants.INTAKE_MOTOR,
            funnelMotor = CoralIntakeConstants.FUNNEL_MOTOR,
            elevatorMotor = CoralIntakeConstants.MASTER_ELEVATOR_MOTOR;
    private final SimpleSensor beamBreak = CoralIntakeConstants.BEAM_BREAK;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(CoralIntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(CoralIntakeConstants.FOC_ENABLED);
    private CoralIntakeConstants.CoralIntakeState targetState = CoralIntakeConstants.CoralIntakeState.REST;

    public CoralIntake() {
        setName("CoralIntake");
    }

    @Override
    public void setBrake(boolean brake) {
        elevatorMotor.setBrake(brake);
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        elevatorMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("CoralElevatorMotor")
                .linearPosition(Units.Meters.of(getCurrentElevatorPositionRotations()))
                .linearVelocity(Units.MetersPerSecond.of(elevatorMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(elevatorMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        CoralIntakeConstants.INTAKE_MECHANISM.update(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        CoralIntakeConstants.FUNNEL_MECHANISM.update(funnelMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        CoralIntakeConstants.ELEVATOR_MECHANISM.update(
                getCurrentElevatorPositionMeters(),
                rotationsToMeters(elevatorMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        Logger.recordOutput("Poses/Components/CoralIntakePose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        intakeMotor.update();
        funnelMotor.update();
        elevatorMotor.update();
        beamBreak.updateSensor();
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return CoralIntakeConstants.ELEVATOR_SYSID_CONFIG;
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
        funnelMotor.stopMotor();
        elevatorMotor.stopMotor();
    }

    public boolean atState(CoralIntakeConstants.CoralIntakeState targetState) {
        return targetState == this.targetState && atTargetAngle();
    }

    public boolean atTargetAngle() {
        final double elevatorDifferenceFromTargetStateRotations = Math.abs(getCurrentElevatorPositionRotations() - targetState.targetPositionRotations);
        return elevatorDifferenceFromTargetStateRotations < CoralIntakeConstants.POSITION_TOLERANCE_ROTATIONS;
    }

    public boolean hasGamePiece() {
        return CoralIntakeConstants.CORAL_COLLECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    /**
     * Checks if a coral has been collected early using the motor's current.
     * This is quicker than {@linkplain CoralIntake#hasGamePiece()} since it updates from the change in current (which happens right when we hit the coral),
     * instead of the beam break which is positioned later on the system.
     *
     * @return whether an early coral collection has been detected
     */
    public boolean isEarlyCoralCollectionDetected() {
        return CoralIntakeConstants.EARLY_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    void setTargetState(CoralIntakeConstants.CoralIntakeState targetState) {
        this.targetState = targetState;

        setTargetState(
                targetState.targetIntakeVoltage,
                targetState.targetFunnelVoltage,
                targetState.targetPositionRotations
        );
    }

    void setTargetState(double targetIntakeVoltage, double targetFunnelVoltage, double targetPositionRotations) {
        setTargetVoltage(targetIntakeVoltage, targetFunnelVoltage);
        setTargetPositionRotations(targetPositionRotations);
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

    private void setTargetPositionRotations(double targetPositionRotations) {
        elevatorMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d originPoint = CoralIntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT;
        final Transform3d intakeTransform = new Transform3d(
                new Translation3d(getCurrentElevatorPositionMeters(), 0, 0),
                new Rotation3d(0, CoralIntakeConstants.INTAKE_ANGLE_FROM_GROUND.getRadians(), 0)
        );

        return originPoint.transformBy(intakeTransform);
    }

    private double getCurrentElevatorPositionMeters() {
        return rotationsToMeters(getCurrentElevatorPositionRotations());
    }

    private double getCurrentElevatorPositionRotations() {
        return elevatorMotor.getSignal(TalonFXSignal.POSITION);
    }

    private double rotationsToMeters(double positionRotations) {
        return Conversions.rotationsToDistance(positionRotations, CoralIntakeConstants.ELEVATOR_DRUM_DIAMETER_METERS);
    }
}