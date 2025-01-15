package frc.trigon.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;
import java.util.function.BiConsumer;


public class GripperCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double targetAngle, Double targetVoltage) -> RobotContainer.GRIPPER.setTargetState(Rotation2d.fromDegrees(targetAngle),targetVoltage),
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

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetGrippingVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetState(targetAngle, targetGrippingVoltage),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }
}