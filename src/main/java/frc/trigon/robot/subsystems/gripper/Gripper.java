package frc.trigon.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.flippable.FlippablePose2d;

public class Gripper extends MotorSubsystem {
    private final TalonFXMotor grippingMotor = GripperConstants.GRIPPING_MOTOR;
    private final TalonFXMotor angleMotor = GripperConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = GripperConstants.ANGLE_ENCODER;
    private final SimpleSensor beamBreak = GripperConstants.BEAM_BREAK;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(GripperConstants.FOC_ENABLED);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withMaxAbsDutyCycle(0.5);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, GripperConstants.DEFAULT_MAXIMUM_VELOCITY, GripperConstants.DEFAULT_MAXIMUM_ACCELERATION, GripperConstants.DEFAULT_MAXIMUM_ACCELERATION * 10).withEnableFOC(GripperConstants.FOC_ENABLED);
    private GripperConstants.GripperState targetState = GripperConstants.GripperState.REST;
    private Rotation2d targetAngle = targetState.targetAngle;

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

    public void setTargetAngleToL4(FlippablePose2d targetScoringPose) {
        this.targetState = GripperConstants.GripperState.SCORE_L4_CLOSE;
        this.targetAngle = calculateTargetAngleForL4(targetScoringPose.get());

        setTargetAngle(this.targetAngle);
    }

    private Rotation2d calculateTargetAngleForL4(Pose2d targetScoringPose) {
        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose();
        var x = currentPose.relativeTo(targetScoringPose);
        final double xDistance = x.getX();
        return calculateTargetAngleForL4(xDistance);
    }

    private Rotation2d calculateTargetAngleForL4(double xDistanceFromScorePose) {
        Logger.recordOutput("XDistanceFromScoredPose", xDistanceFromScorePose);
        final double t = xDistanceFromScorePose / GripperConstants.SCORE_L4_FAR_DISTANCE_METERS;
        return GripperConstants.GripperState.SCORE_L4_CLOSE.targetAngle.interpolate(GripperConstants.GripperState.SCORE_L4_FAR.targetAngle, t);
    }

    public boolean isEjecting() {
        return grippingMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) <= GripperConstants.MINIMUM_VOLTAGE_FOR_EJECTING;
    }

    public boolean isReleasingAlgae() {
        return grippingMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) > -2;
    }

    public boolean isOpenForElevator() {
        return getCurrentEncoderAngle().getDegrees() > GripperConstants.MINIMUM_OPEN_FOR_ELEVATOR_ANGLE.getDegrees();
    }

    public Translation3d calculateAlgaeReleaseVelocity() {
        return new Translation3d(3, 0, 0).rotateBy(new Rotation3d(0, getCurrentEncoderAngle().getRadians() + 2, 0));
    }

    @AutoLogOutput(key = "Gripper/HasGamePiece")
    public boolean hasGamePiece() {
        return GripperConstants.COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    public boolean atState(GripperConstants.GripperState targetState) {
        return targetState == this.targetState && atTargetAngle();
    }

    @AutoLogOutput(key = "Gripper/AtTargetAngle")
    public boolean atTargetAngle() {
        final double currentToTargetStateDifference = Math.abs(getCurrentEncoderAngle().getDegrees() - targetAngle.getDegrees());
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
        return new Translation3d(-5, new Rotation3d(0, -getCurrentEncoderAngle().getRadians(), 0));
    }

    @AutoLogOutput
    public boolean isMovingSlowly() {
        return Math.abs(grippingMotor.getSignal(TalonFXSignal.VELOCITY)) < 4 || (RobotHardwareStats.isSimulation() && SimulationFieldHandler.isHoldingAlgae());
    }

    public Pose3d calculateAlgaeCollectionPose() {
        return calculateVisualizationPose()
                .transformBy(GripperConstants.GRIPPER_TO_HELD_ALGAE);
    }

    void prepareForState(GripperConstants.GripperState targetState) {
        this.targetState = targetState;
        this.targetAngle = targetState.targetAngle;

        setTargetState(
                targetState.targetAngle,
                0
        );
    }

    void setTargetState(GripperConstants.GripperState targetState) {
        this.targetState = targetState;
        this.targetAngle = targetState.targetAngle;

        setTargetState(
                targetState.targetAngle,
                targetState.targetGripperVoltage
        );
    }

    void setTargetState(Rotation2d targetAngle, double targetGrippingVoltage) {
        positionRequest.Slot = 0;
        scalePositionRequestSpeed(targetState.speedScalar);
        setTargetAngle(targetAngle);
        setTargetVoltage(targetGrippingVoltage);
    }

    void setTargetStateWhileHoldingAlgae(Rotation2d targetAngle, double targetGrippingVoltage) {
        positionRequest.Slot = 1;
        scalePositionRequestSpeed(targetState.speedScalar);
        setTargetAngle(targetAngle);
        setTargetVoltage(targetGrippingVoltage);
    }

    private void scalePositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = GripperConstants.DEFAULT_MAXIMUM_VELOCITY * speedScalar;
        positionRequest.Acceleration = GripperConstants.DEFAULT_MAXIMUM_ACCELERATION * speedScalar;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations() - GripperConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET));
    }

    void setTargetVoltage(double targetGrippingVoltage) {
        GripperConstants.GRIPPING_MECHANISM.setTargetVelocity(targetGrippingVoltage);
        grippingMotor.setControl(voltageRequest.withOutput(targetGrippingVoltage));
    }

    public Pose3d calculateVisualizationPose() {
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

    public Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION) + GripperConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET);
    }

    @AutoLogOutput(key = "Gripper/GrippingWheelVelocityMeters")
    private double getGrippingWheelVelocityMetersPerSecond() {
        return Conversions.rotationsToDistance(grippingMotor.getSignal(TalonFXSignal.VELOCITY), GripperConstants.WHEEL_DIAMETER_METERS);
    }
}