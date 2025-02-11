package frc.trigon.robot.subsystems.coralintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.commandfactories.CoralPlacingCommands;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class CoralIntake extends MotorSubsystem {
    private final TalonFXMotor
            intakeMotor = CoralIntakeConstants.INTAKE_MOTOR,
            funnelMotor = CoralIntakeConstants.FUNNEL_MOTOR,
            angleMotor = CoralIntakeConstants.ANGLE_MOTOR;
    private final CANcoderEncoder angleEncoder = CoralIntakeConstants.ANGLE_ENCODER;
    private final SimpleSensor
            beamBreak = CoralIntakeConstants.BEAM_BREAK,
            distanceSensor = CoralIntakeConstants.DISTANCE_SENSOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(CoralIntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(CoralIntakeConstants.FOC_ENABLED);
    private CoralIntakeConstants.CoralIntakeState targetState = CoralIntakeConstants.CoralIntakeState.REST;

    public CoralIntake() {
        setName("CoralIntake");
    }

    @Override
    public void setBrake(boolean brake) {
        angleMotor.setBrake(brake);
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        angleMotor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("CoralAngleMotor")
                .angularPosition(Units.Rotations.of(angleMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(angleMotor.getSignal(TalonFXSignal.ROTOR_VELOCITY) / CoralIntakeConstants.ANGLE_MOTOR_GEAR_RATIO))
                .voltage(Units.Volts.of(angleMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        CoralIntakeConstants.INTAKE_MECHANISM.update(intakeMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        CoralIntakeConstants.FUNNEL_MECHANISM.update(funnelMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        CoralIntakeConstants.ANGLE_MECHANISM.update(
                getCurrentEncoderAngle(),
                Rotation2d.fromRotations(angleMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + CoralIntakeConstants.ANGLE_ENCODER_POSITION_OFFSET_VALUE)
        );

        Logger.recordOutput("Poses/Components/CoralIntakePose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        intakeMotor.update();
        funnelMotor.update();
        angleMotor.update();
        angleEncoder.update();
        beamBreak.updateSensor();
        distanceSensor.updateSensor();
        logForQDashboard();
        Logger.recordOutput("CoralIntake/CurrentAngleDegrees", getCurrentEncoderAngle().getDegrees());
        Logger.recordOutput("CoralIntake/DistanceSensorDetectedDistanceCentimeters", distanceSensor.getScaledValue());
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return CoralIntakeConstants.ANGLE_SYSID_CONFIG;
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
        funnelMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public boolean atState(CoralIntakeConstants.CoralIntakeState targetState) {
        return targetState == this.targetState && atTargetAngle();
    }

    @AutoLogOutput(key = "CoralIntake/AtTargetAngle")
    public boolean atTargetAngle() {
        final double angleDifferenceFromTargetAngleDegrees = Math.abs(getCurrentEncoderAngle().minus(targetState.targetAngle).getDegrees());
        return angleDifferenceFromTargetAngleDegrees < CoralIntakeConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public boolean hasGamePiece() {
        return CoralIntakeConstants.CORAL_COLLECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    public boolean hasGamePieceQuickCheck() {
        return beamBreak.getBinaryValue();
    }

    public boolean isEarlyCoralCollectionDetected() {
        return CoralIntakeConstants.EARLY_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    /**
     * Calculates the pose where the coral should actually be collected from relative to the robot.
     *
     * @return the pose
     */
    public Pose3d calculateCoralCollectionPose() {
        return calculateVisualizationPose()
                .transformBy(getVisualizationToRealPitchTransform())
                .transformBy(CoralIntakeConstants.CORAL_INTAKE_ORIGIN_POINT_TO_CORAL_COLLECTION_TRANSFORM);
    }

    /**
     * Calculates the pose where the coral should rest inside the robot after intaking.
     *
     * @return the pose
     */
    public Pose3d calculateCollectedCoralPose() {
        return calculateVisualizationPose()
                .transformBy(getVisualizationToRealPitchTransform())
                .transformBy(CoralIntakeConstants.CORAL_INTAKE_ORIGIN_POINT_TO_CORAL_VISUALIZATION_TRANSFORM);
    }

    void prepareForState(CoralIntakeConstants.CoralIntakeState targetState) {
        this.targetState = targetState;

        setTargetState(
                0,
                0,
                targetState.targetAngle
        );
    }

    void setTargetState(CoralIntakeConstants.CoralIntakeState targetState) {
        this.targetState = targetState;

        setTargetState(
                targetState.targetIntakeVoltage,
                targetState.targetFunnelVoltage,
                targetState.targetAngle
        );
    }

    void setTargetState(double targetIntakeVoltage, double targetFunnelVoltage, Rotation2d targetAngle) {
        setTargetVoltage(targetIntakeVoltage, targetFunnelVoltage);
        setTargetAngle(targetAngle);
    }

    void setTargetVoltage(double targetIntakeVoltage, double targetFunnelVoltage) {
        setTargetIntakeVoltage(targetIntakeVoltage);
        setTargetFunnelVoltage(targetFunnelVoltage);
    }

    private void setTargetIntakeVoltage(double targetVoltage) {
        CoralIntakeConstants.INTAKE_MECHANISM.setTargetVelocity(targetVoltage);
        intakeMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private void setTargetFunnelVoltage(double targetVoltage) {
        CoralIntakeConstants.FUNNEL_MECHANISM.setTargetVelocity(targetVoltage);
        funnelMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(positionRequest.withPosition(targetAngle.getRotations() - CoralIntakeConstants.ANGLE_ENCODER_POSITION_OFFSET_VALUE));
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d intakeTransform = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, -getCurrentEncoderAngle().getRadians(), 0)
        );

        return CoralIntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.transformBy(intakeTransform);
    }

    private Transform3d getVisualizationToRealPitchTransform() {
        return new Transform3d(
                new Translation3d(),
                CoralIntakeConstants.INTAKE_VISUALIZATION_ORIGIN_POINT.getRotation().unaryMinus()
        );
    }

    private Rotation2d getCurrentEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION) + CoralIntakeConstants.ANGLE_ENCODER_POSITION_OFFSET_VALUE);
    }

    /**
     * Logs the current match time and target reef placement for QDashboard.
     * We use {@link SmartDashboard} instead of {@link Logger} because the {@link Logger} inputs don't show up in QDashboard for some reason.
     */
    private void logForQDashboard() {
        SmartDashboard.putNumber("GameTime", DriverStation.getMatchTime()); // this is called gameTime instead of matchTime because MatchTime doesn't show up in QDashboard for some reason
        logTargetPlacementStates();
    }

    private void logTargetPlacementStates() {
        final boolean[] targetReefSideArray = new boolean[12];
        final int targetReefPositionIndex = calculateTargetReefPositionQDashboardIndex();
        targetReefSideArray[targetReefPositionIndex] = true;

        SmartDashboard.putBooleanArray("TargetCoralPlacementStates/TargetReefSideArray", targetReefSideArray);
        SmartDashboard.putNumber("TargetCoralPlacementStates/TargetLevel", CoralPlacingCommands.TARGET_SCORING_LEVEL.level);
    }

    private int calculateTargetReefPositionQDashboardIndex() {
        final int qDashboardIndexBeforeSideAccountability = CoralPlacingCommands.TARGET_REEF_SCORING_CLOCK_POSITION.qDashboardOrder * 2;
        return CoralPlacingCommands.TARGET_REEF_SCORING_SIDE.doesFlipYTransformWhenFacingDriverStation ? qDashboardIndexBeforeSideAccountability + 1 : qDashboardIndexBeforeSideAccountability;
    }
}