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
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor motor = ElevatorConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.REST;

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
                .linearPosition(Units.Meters.of(getPositionRotations()))
                .linearVelocity(Units.MetersPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        Logger.recordOutput("Poses/Components/ElevatorFirstPose", getElevatorFirstComponentPose());
        Logger.recordOutput("Poses/Components/ElevatorSecondPose", getElevatorSecondComponentPose());

        ElevatorConstants.MECHANISM.update(
                rotationsToMeters(getPositionRotations()),
                rotationsToMeters(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void sysIdDrive(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public boolean atTargetState() {
        final double currentToTargetStateDifference = Math.abs(targetState.targetPositionMeters - getPositionMeters());
        return currentToTargetStateDifference < ElevatorConstants.TOLERANCE_METERS;
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;
        setTargetPositionRotations(metersToRotations(targetState.targetPositionMeters));
    }

    void setTargetPositionRotations(double targetPositionRotations) {
        motor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    private Pose3d getElevatorFirstComponentPose() {
        final Pose3d originPoint = ElevatorConstants.ELEVATOR_VISUALIZATION_ORIGIN_POINT;

        if (elevatorDidNotExtendFirstComponentLength())
            return originPoint.transformBy(getCurrentElevatorTransform(ElevatorConstants.FIRST_ELEVATOR_COMPONENT_HEIGHT_DIFFERANCE_FROM_ROBOT));
        return originPoint.transformBy(
                createNewTransform(ElevatorConstants.FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH)
        );
    }

    private Pose3d getElevatorSecondComponentPose() {
        final Pose3d originPoint = ElevatorConstants.SECOND_ELEVATOR_VISUALIZATION_ORIGIN_POINT;
        return originPoint.transformBy(getCurrentElevatorTransform(ElevatorConstants.SECOND_ELEVATOR_COMPONENT_HEIGHT_DIFFERANCE_FROM_FIRST_COMPONENT));
    }

    private boolean elevatorDidNotExtendFirstComponentLength() {
        return getPositionMeters() < ElevatorConstants.FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH;
    }

    private Transform3d getCurrentElevatorTransform(double heightAboveLowerComponent) {
        return createNewTransform(getPositionMeters() + heightAboveLowerComponent);
    }

    private Transform3d createNewTransform(double transformHeight) {
        return new Transform3d(
                new Translation3d(0, 0, transformHeight),
                new Rotation3d()
        );
    }

    private double getPositionRotations() {
        return motor.getSignal(TalonFXSignal.POSITION);
    }

    private double getPositionMeters() {
        return rotationsToMeters(getPositionRotations());
    }

    private double rotationsToMeters(double positionRotations) {
        return Conversions.rotationsToDistance(positionRotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double metersToRotations(double positionMeters) {
        return Conversions.distanceToRotations(positionMeters, ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}