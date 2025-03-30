package frc.trigon.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;
import org.trigon.utilities.flippable.FlippablePose2d;

import java.util.Set;
import java.util.function.Supplier;


public class GripperCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees, targetVoltage) -> RobotContainer.GRIPPER.setTargetState(Rotation2d.fromDegrees(targetAngleDegrees), targetVoltage),
                false,
                Set.of(RobotContainer.GRIPPER),
                "Debugging/GripperTargetAngleDegrees",
                "Debugging/GripperTargetGrippingVoltage"
        );
    }

    public static Command getGearRatioCalulationCommand() {
        return new GearRatioCalculationCommand(
                GripperConstants.ANGLE_MOTOR,
                GripperConstants.ANGLE_ENCODER,
                0.5,
                RobotContainer.GRIPPER
        );
    }

    public static Command getGripperDefaultCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getSetTargetStateCommand(GripperConstants.GripperState.OPEN_FOR_NOT_HITTING_REEF),
                getSetTargetStateCommand(GripperConstants.GripperState.REST),
                () -> RobotContainer.ELEVATOR.willCurrentMovementMoveThroughHitRange() || RobotContainer.GRIPPER.hasGamePiece()
        );
    }

    public static Command getSetTargetStateWithCurrentCommand(GripperConstants.GripperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetStateWithCurrent(targetState.targetAngle, targetState.targetGripperVoltage),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }

    public static Command getPrepareForStateCommand(Supplier<GripperConstants.GripperState> targetStateSupplier) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.prepareForState(targetStateSupplier.get()),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }

    public static Command getPrepareForStateCommand(GripperConstants.GripperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.prepareForState(targetState),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }

    public static Command getSetTargetStateCommand(Supplier<GripperConstants.GripperState> targetStateSupplier) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetState(targetStateSupplier.get()),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }

    public static Command getSetTargetStateCommand(GripperConstants.GripperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetState(targetState),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }

    public static Command getPrepareForScoringInL4Command(Supplier<FlippablePose2d> targetPoseSupplier) {
        return new ExecuteEndCommand(
                () -> {
                    RobotContainer.GRIPPER.setTargetAngleToL4(targetPoseSupplier.get());
                    RobotContainer.GRIPPER.setTargetVoltage(0);
                },
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }

    public static Command getScoreInL4Command(Supplier<FlippablePose2d> targetPoseSupplier) {
        return new ExecuteEndCommand(
                () -> {
                    RobotContainer.GRIPPER.setTargetAngleToL4(targetPoseSupplier.get());
                    RobotContainer.GRIPPER.setTargetVoltage(GripperConstants.GripperState.SCORE_L4_CLOSE.targetGripperVoltage);
                },
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetGrippingVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetState(targetAngle, targetGrippingVoltage),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }
}