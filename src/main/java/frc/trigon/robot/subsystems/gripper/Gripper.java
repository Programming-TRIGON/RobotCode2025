package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Gripper extends MotorSubsystem {
    private final TalonFXMotor motor = GripperConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(GripperConstants.FOC_ENABLED);

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateMechanism() {
        GripperConstants.GRIPPER_MECHANISM.update(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }

    void stopMotor() {
        motor.stopMotor();
    }

    void setTargetVoltage(GripperConstants.GripperState targetVoltage) {
        GripperConstants.GRIPPER_MECHANISM.setTargetVelocity(targetVoltage.targetVoltage);
        motor.setControl(voltageRequest.withOutput(targetVoltage.targetVoltage));
    }
}