package frc.trigon.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class GripperCommands {
    public static Command getSetTargetStateCommand(GripperConstants.GripperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetState(targetState),
                RobotContainer.GRIPPER::stopMotor,
                RobotContainer.GRIPPER
        );
    }
    private static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetVoltage(targetVoltage),
                RobotContainer.GRIPPER::stopMotor,
                RobotContainer.GRIPPER
        );
    }
}