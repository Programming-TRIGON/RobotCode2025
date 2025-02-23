package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor motor = ClimberConstants.MOTOR;
    private final DynamicMotionMagicVoltage
            groundedPositionRequest = new DynamicMotionMagicVoltage(
            0,
            ClimberConstants.MAXIMUM_GROUNDED_VELOCITY,
            ClimberConstants.MAXIMUM_GROUNDED_ACCELERATION,
            0
    ).withSlot(ClimberConstants.GROUNDED_PID_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC),
            onCagePositionRequest = new DynamicMotionMagicVoltage(
                    0,
                    ClimberConstants.MAXIMUM_ON_CAGE_VELOCITY,
                    ClimberConstants.MAXIMUM_ON_CAGE_ACCELERATION,
                    0
            ).withSlot(ClimberConstants.ON_CAGE_PID_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private ClimberConstants.ClimberState targetState = ClimberConstants.ClimberState.REST;
    private boolean isClimbing = false;

    public Climber() {
        setName("Climber");
        GeneralCommands.getDelayedCommand(3, this::configureChangingDefaultCommand).schedule();
    }

    @Override
    public void sysIdDrive(double targetDrivePower) {
        motor.setControl(voltageRequest.withOutput(targetDrivePower));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("ClimberMotor")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void updateMechanism() {
        ClimberConstants.CLIMBER_VISUALIZATION.update(
                motor.getSignal(TalonFXSignal.POSITION),
                motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)
        );
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public boolean atTargetState() {
        return atState(targetState);
    }

    public boolean atState(ClimberConstants.ClimberState state) {
        return Math.abs(motor.getSignal(TalonFXSignal.POSITION) - state.positionRotations) < ClimberConstants.CLIMBER_TOLERANCE_METERS;
    }

    public void setIsClimbing(boolean isClimbing) {
        this.isClimbing = isClimbing;
        Logger.recordOutput("IsClimbing", isClimbing);
    }

    public boolean isClimbing() {
        return isClimbing;
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        this.targetState = targetState;
        setTargetPosition(targetState.positionRotations, targetState.isAffectedByRobotWeight);
    }

    void setTargetPosition(double positionRotations, boolean isAffectedByRobotWeight) {
        motor.setControl(determineRequest(isAffectedByRobotWeight).withPosition(positionRotations));
    }

    private DynamicMotionMagicVoltage determineRequest(boolean isAffectedByRobotWeight) {
        return isAffectedByRobotWeight ? onCagePositionRequest : groundedPositionRequest;
    }

    private void configureChangingDefaultCommand() {
        final Trigger climberDefaultCommandTrigger = new Trigger(() -> isClimbing);
        climberDefaultCommandTrigger.onTrue(new InstantCommand(this::defaultToClimbing));
        climberDefaultCommandTrigger.onFalse(new InstantCommand(this::defaultToBraking));
    }

    private void defaultToBraking() {
        changeDefaultCommand(ClimberCommands.getStopCommand());
    }

    private void defaultToClimbing() {
        changeDefaultCommand(ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB));
    }
}