package frc.trigon.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

import java.util.Set;


public class GripperCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (Double[] targetStates) -> RobotContainer.GRIPPER.setTargetState(
                        Rotation2d.fromDegrees(targetStates[0]),
                        targetStates[1]
                ),
                false,
                Set.of(RobotContainer.GRIPPER),
                "Debugging/GripperTargetAngleRotations",
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

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetState(targetAngle, targetVoltage),
                RobotContainer.GRIPPER::stop,
                RobotContainer.GRIPPER
        );
    }
}