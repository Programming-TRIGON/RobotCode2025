package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.grapple.lasercan.LaserCAN;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Gripper extends MotorSubsystem {
    private final TalonFXMotor grippingMotor = GripperConstants.GRIPPING_MOTOR;
    private final TalonFXMotor angleMotor = GripperConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = GripperConstants.ANGLE_ENCODER;
    private final LaserCAN laserCAN = GripperConstants.LASER_CAN;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(GripperConstants.FOC_ENABLED);
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
        laserCAN.update();
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
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        GripperConstants.GRIPPING_MECHANISM.update(grippingMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        GripperConstants.ANGLE_MECHANISM.update(
                getCurrentEncoderAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        Logger.recordOutput("Poses/Components/GripperPose", calculateVisualizationPose());
    }

    public boolean hasGamePiece() {
        return laserCAN.hasResult() && laserCAN.getDistanceMillimeters() < GripperConstants.GAME_PIECE_DETECTION_THRESHOLD_MILLIMETERS;
    }

    public boolean atState(GripperConstants.GripperState targetState) {
        return targetState == this.targetState && atTargetAngle();
    }

    public boolean atTargetAngle() {
        final double currentToTargetStateDifference = Math.abs(getCurrentEncoderAngle().getDegrees() - targetState.targetAngle.getDegrees());
        return currentToTargetStateDifference < GripperConstants.POSITION_TOLERANCE_DEGREES.getDegrees();
    }

    public Pose3d calculateCoralReleasePoint() {
        return calculateVisualizationPose().transformBy(GripperConstants.GRIPPER_TO_CORAL_RELEASE);
    }

    public Translation2d getRobotRelativeExitXYVelocity() {
        return new Translation2d(getGrippingWheelVelocityMetersPerSecond(), 0).times(getCurrentEncoderAngle().getCos());
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

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations()));
    }

    private void setTargetVoltage(double targetGrippingVoltage) {
        GripperConstants.GRIPPING_MECHANISM.setTargetVelocity(targetGrippingVoltage);
        grippingMotor.setControl(voltageRequest.withOutput(targetGrippingVoltage));
    }

    private Pose3d calculateVisualizationPose() {
        final Pose3d currentElevatorPose = RobotContainer.ELEVATOR.getFirstStageComponentPose();
        final Transform3d elevatorToGripper = GripperConstants.ELEVATOR_TO_GRIPPER;
        final Pose3d gripperOrigin = currentElevatorPose.transformBy(elevatorToGripper);

        final Transform3d transform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, getCurrentEncoderAngle().getRadians(), 0)
        );
        return gripperOrigin.transformBy(transform);
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }

    private double getGrippingWheelVelocityMetersPerSecond() {
        return Conversions.rotationsToDistance(grippingMotor.getSignal(TalonFXSignal.VELOCITY), GripperConstants.WHEEL_DIAMETER_METERS);
    }
}