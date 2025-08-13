package frc.trigon.robot.subsystems.coralintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.commandfactories.CoralCollectionCommands;
import frc.trigon.robot.commands.commandfactories.CoralPlacingCommands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
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
    private final LoggedNetworkBoolean overrideCoralCollectionDetection = new LoggedNetworkBoolean("/SmartDashboard/OverrideCoralCollection", false);
    private CoralIntakeConstants.CoralIntakeState targetState = CoralIntakeConstants.CoralIntakeState.REST;
    private final Timer pulsingTimer = new Timer();
    private boolean isPulsingOn = false;

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
        CoralIntakeConstants.BACKUP_BEAM_BREAK.updateSensor();
        distanceSensor.updateSensor();
        logToDashboard();
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

    public CoralIntakeConstants.CoralIntakeState getTargetState() {
        return targetState;
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
        return overrideCoralCollectionDetection.get() ?
                CoralIntakeConstants.OVERRIDE_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean() :
                CoralIntakeConstants.CORAL_COLLECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    public boolean isEarlyCoralCollectionDetected() {
        return CoralIntakeConstants.EARLY_CORAL_COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    @AutoLogOutput
    public boolean isMovingSlowly() {
        return Math.abs(intakeMotor.getSignal(TalonFXSignal.VELOCITY)) < 4;
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

    void initializePulsing() {
        pulsingTimer.restart();
        isPulsingOn = true;
    }

    void pulseIntake() {
        final double pulsingTime = isPulsingOn ? CoralIntakeConstants.PULSING_ON_PERIOD_SECONDS : CoralIntakeConstants.PULSING_OFF_PERIOD_SECONDS;
        if (pulsingTimer.advanceIfElapsed(pulsingTime) && getCurrentEncoderAngle().getDegrees() > CoralIntakeConstants.PULSING_ANGLE_DEGREES)
            isPulsingOn = !isPulsingOn;

        setTargetState(
                isPulsingOn ? CoralIntakeConstants.PULSING_INTAKE_MOTOR_VOLTAGE : 0,
                CoralIntakeConstants.PULSING_FUNNEL_MOTOR_VOLTAGE,
                CoralIntakeConstants.CoralIntakeState.LOAD_CORAL_TO_GRIPPER_SEEING_GAME_PIECE_WITH_BEAM_BREAK.targetAngle
        );
    }

    void stopPulsing() {
        pulsingTimer.stop();
        stop();
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

    private void logToDashboard() {
        Logger.recordOutput("GameTime", DriverStation.getMatchTime());
        Logger.recordOutput("TargetCoralPlacementStates/IsTargetSideLeft", !OperatorConstants.REEF_CHOOSER.getReefSide().doesFlipYTransformWhenFacingDriverStation);
        Logger.recordOutput("TargetCoralPlacementStates/TargetLevel", OperatorConstants.REEF_CHOOSER.getScoringLevel().level);
        Logger.recordOutput("TargetCoralPlacementStates/TargetClockPositionForElastic", getClockPositionForElastic());
        Logger.recordOutput("ShouldIgnoreLollipopCoral", CoralCollectionCommands.SHOULD_IGNORE_LOLLIPOP_CORAL);
        Logger.recordOutput("ShouldAssistIntake", CoralCollectionCommands.SHOULD_ASSIST_INTAKE);
        Logger.recordOutput("ShouldKeepIntakeOpen", CoralCollectionCommands.SHOULD_KEEP_INTAKE_OPEN);
        Logger.recordOutput("ShouldScoreCoralAutonomously", CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY);
    }

    private double getClockPositionForElastic() {
        if (OperatorConstants.REEF_CHOOSER.getReefSide().doesFlipYTransformWhenFacingDriverStation)
            return OperatorConstants.REEF_CHOOSER.getClockPosition().clockPosition - 0.5;
        return OperatorConstants.REEF_CHOOSER.getClockPosition().clockPosition + 0.5;
    }
}