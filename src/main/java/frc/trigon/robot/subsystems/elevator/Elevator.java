package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor motor = ElevatorConstants.MASTER_MOTOR;
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY,
            ElevatorConstants.MOTION_MAGIC_ACCELERATION,
            0
    ).withUpdateFreqHz(100).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED).withUpdateFreqHz(100);

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
        ElevatorConstants.MECHANISM.update(getPositionMeters(), toMeters(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
    }


    void setTargetElevatorState(ElevatorConstants.ElevatorState targetState) {
        setTargetPosition(targetState.positionMeters);
    }

    void setTargetPosition(double targetPositionMeters) {
        motor.setControl(positionRequest.withPosition(toRotations(targetPositionMeters)));
    }

    private double getPositionMeters() {
        return toMeters(motor.getSignal(TalonFXSignal.POSITION));
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double toRotations(double meters) {
        return Conversions.distanceToRotations(meters, ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}

