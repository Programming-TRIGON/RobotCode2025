package frc.trigon.robot.subsystems.algaemanipulator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class AlgaeManipulator extends MotorSubsystem {
    private final TalonFXMotor angleMotor = AlgaeManipulatorConstants.ANGLE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(AlgaeManipulatorConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(AlgaeManipulatorConstants.FOC_ENABLED);
    private AlgaeManipulatorConstants.AlgaeManipulatorState targetState = AlgaeManipulatorConstants.AlgaeManipulatorState.REST;

    public AlgaeManipulator() {
        setName("AlgaeManipulator");
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("AlgaeManipulator")
                .angularPosition(Units.Rotations.of(getCurrentAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return AlgaeManipulatorConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        angleMotor.update();
    }

    @Override
    public void updateMechanism() {
        AlgaeManipulatorConstants.ANGLE_MECHANISM.update(
                getCurrentAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    @Override
    public void stop() {
        angleMotor.stopMotor();
    }

    public boolean atState(AlgaeManipulatorConstants.AlgaeManipulatorState state) {
        return targetState == state && atTargetAngle();
    }

    @AutoLogOutput(key = "AlgaeManipulator/AtTargetAngle")
    public boolean atTargetAngle() {
        final double difference = Math.abs(getCurrentAngle().getDegrees() - targetState.targetAngle.getDegrees());
        return difference < AlgaeManipulatorConstants.ANGLE_TOLERANCE.getDegrees();
    }

    void setTargetState(AlgaeManipulatorConstants.AlgaeManipulatorState targetState) {
        this.targetState = targetState;
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.POSITION));
    }
}