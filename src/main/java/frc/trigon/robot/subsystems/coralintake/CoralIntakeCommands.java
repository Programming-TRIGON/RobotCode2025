package frc.trigon.robot.subsystems.coralintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.CoralCollectionCommands;
import frc.trigon.robot.commands.commandfactories.CoralPlacingCommands;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class CoralIntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double[] targetStates) -> RobotContainer.CORAL_INTAKE.setTargetState(
                        targetStates[0],
                        targetStates[1],
                        Rotation2d.fromDegrees(targetStates[2])
                ),
                false,
                Set.of(RobotContainer.CORAL_INTAKE),
                "Debugging/TargetCoralIntakeVoltage",
                "Debugging/TargetCoralIntakeFunnelVoltage",
                "Debugging/TargetCoralIntakeAngleDegrees"
        );
    }

    public static Command getGearRatioCalculationCommand() {
        return new GearRatioCalculationCommand(
                CoralIntakeConstants.ANGLE_MOTOR,
                CoralIntakeConstants.ANGLE_ENCODER,
                0.08,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getCoralIntakeDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_FROM_FLOOR),
                getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.REST),
                () -> CoralCollectionCommands.SHOULD_KEEP_INTAKE_OPEN && !OperatorConstants.REEF_CHOOSER.getScoringLevel().equals(CoralPlacingCommands.ScoringLevel.L1_CORAL_INTAKE)
        );
    }

    public static Command getCenterCoralWithPulsingCommand() {
        return new FunctionalCommand(
                RobotContainer.CORAL_INTAKE::initializePulsing,
                RobotContainer.CORAL_INTAKE::pulseIntake,
                (interrupted) -> RobotContainer.CORAL_INTAKE.stopPulsing(),
                () -> false,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.prepareForState(targetState),
                () -> {
                },
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(targetState),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }

    public static Command getSetTargetStateCommand(double targetIntakeVoltage, double targetFunnelVoltage, Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.CORAL_INTAKE.setTargetState(targetIntakeVoltage, targetFunnelVoltage, targetAngle),
                RobotContainer.CORAL_INTAKE::stop,
                RobotContainer.CORAL_INTAKE
        );
    }
}