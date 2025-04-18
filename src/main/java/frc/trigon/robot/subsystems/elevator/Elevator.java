package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = ElevatorConstants.MASTER_MOTOR,
            followerMotor = ElevatorConstants.FOLLOWER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, ElevatorConstants.DEFAULT_MAXIMUM_VELOCITY, ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION, ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION * 10).withEnableFOC(ElevatorConstants.FOC_ENABLED);
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
        masterMotor.setBrake(brake);
        followerMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .linearPosition(Units.Meters.of(getPositionRotations()))
                .linearVelocity(Units.MetersPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        Logger.recordOutput("Poses/Components/ElevatorFirstPose", getFirstStageComponentPose());
        Logger.recordOutput("Poses/Components/ElevatorSecondPose", getSecondStageComponentPose());

        ElevatorConstants.MECHANISM.update(
                getPositionMeters(),
                rotationsToMeters(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        Logger.recordOutput("Elevator/CurrentPositionMeters", getPositionMeters());
    }

    @Override
    public void sysIdDrive(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public boolean willCurrentMovementMoveThroughHitRange() {
        return willMovementMoveThroughHitRange(metersToRotations(targetState.targetPositionMeters));
    }

    public boolean isElevatorOverAlgaeHitRange() {
        return getPositionRotations() > ElevatorConstants.GRIPPER_HITTING_ALGAE_UPPER_BOUND_POSITION_ROTATIONS;
    }

    public Pose3d getFirstStageComponentPose() {
        return calculateCurrentElevatorPoseFromOrigin(ElevatorConstants.FIRST_STAGE_VISUALIZATION_ORIGIN_POINT);
    }

    public boolean atState(ElevatorConstants.ElevatorState targetState) {
        return targetState == this.targetState && atTargetState();
    }

    @AutoLogOutput(key = "Elevator/AtTargetState")
    public boolean atTargetState() {
        final double currentToTargetStateDifference = Math.abs(targetState.targetPositionMeters - getPositionMeters());
        return currentToTargetStateDifference < ElevatorConstants.POSITION_TOLERANCE_METERS;
    }

    public double metersToRotations(double positionMeters) {
        return Conversions.distanceToRotations(positionMeters, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;
        scalePositionRequestSpeed(targetState.speedScalar);
        setTargetPositionRotations(metersToRotations(targetState.targetPositionMeters));
    }

    private void scalePositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = ElevatorConstants.DEFAULT_MAXIMUM_VELOCITY * speedScalar;
        positionRequest.Acceleration = ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION * speedScalar;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    void setTargetPositionRotations(double targetPositionRotations) {
        masterMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    boolean isAllowedToMoveToPosition(double targetPositionRotations) {
        if (!willMovementMoveThroughHitRange(targetPositionRotations))
            return true;

        return RobotContainer.GRIPPER.isOpenForElevator();
    }

    public boolean willMovementMoveThroughHitRange(double targetPositionRotations) {
        final double currentPositionRotations = masterMotor.getSignal(TalonFXSignal.POSITION);
        if (isInGripperHitRange(currentPositionRotations) || isInGripperHitRange(targetPositionRotations))
            return true;

        if (currentPositionRotations > ElevatorConstants.GRIPPER_HITTING_ELEVATOR_BASE_UPPER_BOUND_POSITION_ROTATIONS)
            return targetPositionRotations < ElevatorConstants.GRIPPER_HITTING_ELEVATOR_BASE_UPPER_BOUND_POSITION_ROTATIONS;

        return targetPositionRotations > ElevatorConstants.GRIPPER_HITTING_ELEVATOR_BASE_UPPER_BOUND_POSITION_ROTATIONS;
    }

    private boolean isInGripperHitRange(double positionRotations) {
        return positionRotations > ElevatorConstants.GRIPPER_HITTING_ELEVATOR_BASE_LOWER_BOUND_POSITION_ROTATIONS && positionRotations < ElevatorConstants.GRIPPER_HITTING_ELEVATOR_BASE_UPPER_BOUND_POSITION_ROTATIONS;
    }

    private Pose3d getSecondStageComponentPose() {
        return calculateComponentPose(getSecondPoseHeight(), ElevatorConstants.SECOND_STAGE_VISUALIZATION_ORIGIN_POINT);
    }

    private double getSecondPoseHeight() {
        if (firstComponentLimitReached())
            return getPositionMeters() - ElevatorConstants.FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS;
        return 0;
    }

    private boolean firstComponentLimitReached() {
        return getPositionMeters() > ElevatorConstants.FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS;
    }

    private Pose3d calculateCurrentElevatorPoseFromOrigin(Pose3d originPoint) {
        return calculateComponentPose(getPositionMeters(), originPoint);
    }

    private Pose3d calculateComponentPose(double poseHeight, Pose3d originPoint) {
        final Transform3d elevatorTransform = new Transform3d(
                new Translation3d(0, 0, poseHeight),
                new Rotation3d()
        );
        return originPoint.transformBy(elevatorTransform);
    }

    private double getPositionRotations() {
        return masterMotor.getSignal(TalonFXSignal.POSITION);
    }

    private double getPositionMeters() {
        return rotationsToMeters(getPositionRotations());
    }

    private double rotationsToMeters(double positionRotations) {
        return Conversions.rotationsToDistance(positionRotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}