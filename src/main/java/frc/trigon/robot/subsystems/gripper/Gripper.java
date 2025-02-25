package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Gripper extends MotorSubsystem {
    private final TalonFXMotor grippingMotor = GripperConstants.GRIPPING_MOTOR;
    private final TalonFXMotor angleMotor = GripperConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = GripperConstants.ANGLE_ENCODER;
    private final SimpleSensor beamBreak = GripperConstants.BEAM_BREAK;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(GripperConstants.FOC_ENABLED);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(GripperConstants.FOC_ENABLED);
    private GripperConstants.GripperState targetState = GripperConstants.GripperState.REST;

    public Gripper() {
        setName("Gripper");
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return GripperConstants.SYSID_CONFIG;
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updatePeriodically() {
        grippingMotor.update();
        angleMotor.update();
        angleEncoder.update();
        beamBreak.updateSensor();
        Logger.recordOutput("Gripper/CurrentAngleDegrees", getCurrentEncoderAngle().getDegrees());
    }

    @Override
    public void stop() {
        grippingMotor.stopMotor();
        angleMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("GripperAngleMotor")
                .angularPosition(Units.Rotations.of(angleEncoder.getSignal(CANcoderSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.ROTOR_VELOCITY) / GripperConstants.ANGLE_MOTOR_GEAR_RATIO))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        GripperConstants.GRIPPING_MECHANISM.update(grippingMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        GripperConstants.ANGLE_MECHANISM.update(
                getCurrentEncoderAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + GripperConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET)
        );
        Logger.recordOutput("Poses/Components/GripperPose", calculateVisualizationPose());
    }

    public boolean isEjecting() {
        return grippingMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) <= GripperConstants.MINIMUM_VOLTAGE_FOR_EJECTING;
    }

    public boolean isOpenForElevator() {
        return getCurrentEncoderAngle().getDegrees() > GripperConstants.MINIMUM_OPEN_FOR_ELEVATOR_ANGLE.getDegrees();
    }

    public boolean hasGamePiece() {
        return GripperConstants.COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    public boolean atState(GripperConstants.GripperState targetState) {
        return targetState == this.targetState && atTargetAngle();
    }

    @AutoLogOutput(key = "Gripper/AtTargetAngle")
    public boolean atTargetAngle() {
        final double currentToTargetStateDifference = Math.abs(getCurrentEncoderAngle().getDegrees() - targetState.targetAngle.getDegrees());
        return currentToTargetStateDifference < GripperConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public Pose3d calculateCoralReleasePoint() {
        return calculateVisualizationPose()
                .transformBy(getVisualizationToRealPitchTransform())
                .transformBy(GripperConstants.GRIPPER_TO_CORAL_RELEASE);
    }

    public Pose3d calculateHeldCoralVisualizationPose() {
        return calculateVisualizationPose()
                .transformBy(getVisualizationToRealPitchTransform())
                .transformBy(GripperConstants.GRIPPER_TO_HELD_CORAL);
    }

    public Translation3d getRobotRelativeExitVelocity() {
        return new Translation3d(getGrippingWheelVelocityMetersPerSecond(), new Rotation3d(0, -getCurrentEncoderAngle().getRadians(), 0));
    }

    @AutoLogOutput
    public boolean isMovingSlowly() {
        return -grippingMotor.getSignal(TalonFXSignal.VELOCITY) < 1;
    }

    /**
     * This will set the gripper to the score in reef state,
     * which means the gripper will eject the coral into the reef, while maintaining the current target angle.
     */
    void scoreInReefForAuto() {
        setTargetState(targetState.targetAngle, GripperConstants.SCORE_IN_REEF_FOR_AUTO_VOLTAGE);
    }

    void prepareForState(GripperConstants.GripperState targetState) {
        this.targetState = targetState;

        setTargetState(
                targetState.targetAngle,
                0
        );
    }

    void setTargetState(GripperConstants.GripperState targetState) {
        this.targetState = targetState;

        setTargetState(
                targetState.targetAngle,
                targetState.targetGripperVoltage
        );
    }

    void setTargetState(Rotation2d targetAngle, double targetGrippingVoltage) {
        setTargetAngle(targetAngle);
        setTargetVoltage(targetGrippingVoltage);
    }

    void setTargetStateWithCurrent(Rotation2d targetAngle, double targetGrippingCurrent) {
        setTargetAngle(targetAngle);
        grippingMotor.setControl(torqueCurrentRequest.withOutput(targetGrippingCurrent));
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations() - GripperConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET));
    }

    private void setTargetVoltage(double targetGrippingVoltage) {
        GripperConstants.GRIPPING_MECHANISM.setTargetVelocity(targetGrippingVoltage);
        grippingMotor.setControl(voltageRequest.withOutput(targetGrippingVoltage));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d currentElevatorPose = RobotContainer.ELEVATOR.getFirstStageComponentPose();
        final Pose3d gripperOrigin = currentElevatorPose.transformBy(GripperConstants.ELEVATOR_TO_GRIPPER);

        final Transform3d pitchTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, getCurrentEncoderAngle().getRadians(), 0)
        );
        return gripperOrigin.transformBy(pitchTransform);
    }

    private Transform3d getVisualizationToRealPitchTransform() {
        return new Transform3d(
                new Translation3d(),
                new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-62 + 90), 0)
        );
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION) + GripperConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET);
    }

    @AutoLogOutput(key = "Gripper/GrippingWheelVelocityMeters")
    private double getGrippingWheelVelocityMetersPerSecond() {
        return Conversions.rotationsToDistance(grippingMotor.getSignal(TalonFXSignal.VELOCITY), GripperConstants.WHEEL_DIAMETER_METERS);
    }
}