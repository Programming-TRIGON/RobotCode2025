package frc.trigon.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;

public class GripperCommands {
    public static Command getSetTargetVoltageCommand(GripperConstants.GripperState targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.GRIPPER.setTargetVoltage(targetState),
                RobotContainer.GRIPPER::stopMotor,
                RobotContainer.GRIPPER
        );
    }
}