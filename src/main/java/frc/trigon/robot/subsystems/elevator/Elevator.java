package frc.trigon.robot.subsystems.elevator;

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
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor motor = ElevatorConstants.MASTER_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);

    public Elevator() {
        setName("Elevator");
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
                .linearPosition(Units.Meters.of(motor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        ElevatorConstants.MECHANISM.update(getPositionRotation(), motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        motor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    void setTargetElevatorState(ElevatorConstants.ElevatorState targetState) {
        setTargetPosition(targetState.targetPositionRotations);
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

    private double getPositionRotation() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }

    private double getPositionMeters() {
        return rotationsToMeters(motor.getSignal(TalonFXSignal.POSITION));
    }

    private double rotationsToMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}