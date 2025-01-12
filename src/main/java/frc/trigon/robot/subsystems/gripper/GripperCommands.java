package frc.trigon.robot.subsystems.gripper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;

public class GripperCommands {
    public static Command getSetTargetStateCommand(GripperConstants.GripperState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetState(targetState),
                RobotContainer.GRIPPER::stopMotors,
                RobotContainer.GRIPPER
        );
    }
    private static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.GRIPPER.setTargetVoltage(targetVoltage),
                RobotContainer.GRIPPER::stopMotors,
                RobotContainer.GRIPPER
        );
    }
    private static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new ExecuteEndCommand(
                () -> RobotContainer.GRIPPER.setTargetAngle(targetAngle),
                RobotContainer.GRIPPER::stopMotors,
                RobotContainer.GRIPPER
        );
    }
}