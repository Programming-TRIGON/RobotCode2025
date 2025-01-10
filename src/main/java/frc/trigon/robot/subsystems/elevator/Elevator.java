package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor motor = ElevatorConstants.MASTER_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private ElevatorConstants.ElevatorState currentTargetState = ElevatorConstants.ElevatorState.REST;

    public Elevator() {
        setName("Elevator");
    }

    public boolean atTargetState() {
        return motor.getSignal(TalonFXSignal.POSITION) == currentTargetState.targetPositionRotations;
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ElevatorConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .linearPosition(Units.Meters.of(getPositionMeters()))
                .linearVelocity(Units.MetersPerSecond.of(getPositionMeters()))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.VELOCITY)));
    }

    @Override
    public void updateMechanism() {
        ElevatorConstants.MECHANISM.update(getPositionAngle().getRotations(), motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        motor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        currentTargetState = targetState;
        setTargetPosition(currentTargetState.targetPositionRotations);
    }

    void setTargetPosition(double targetPositionRotations) {
        motor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    private Pose3d getElevatorComponentPose() {
        final Pose3d originPoint = ElevatorConstants.ELEVATOR_ORIGIN_POINT;
        final Transform3d elevatorTransform = new Transform3d(
                new Translation3d(0, 0, getPositionMeters()),
                new Rotation3d()
        );
        return originPoint.transformBy(elevatorTransform);
    }

    private Rotation2d getPositionAngle() {
        return new Rotation2d(motor.getSignal(TalonFXSignal.POSITION));
    }

    private double getPositionMeters() {
        return rotationsToMeters(motor.getSignal(TalonFXSignal.POSITION));
    }

    private double rotationsToMeters(double positionRotations) {
        return Conversions.rotationsToDistance(positionRotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}