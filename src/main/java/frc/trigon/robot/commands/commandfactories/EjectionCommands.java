package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;

public class EjectionCommands {
    public static Command getEjectCoralCommand() {
        return new ConditionalCommand(
                getEjectCoralFromGripperCommand(),
                getEjectCoralFromCoralIntakeCommand(),
                RobotContainer.GRIPPER::hasGamePiece
        );
    }

    private static Command getEjectCoralFromGripperCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getPrepareForStateCommand(GripperConstants.GripperState.EJECT).until(RobotContainer.GRIPPER::atTargetAngle),
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.EJECT)
        );
    }

    private static Command getEjectCoralFromCoralIntakeCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getPrepareForStateCommand(CoralIntakeConstants.CoralIntakeState.EJECT).until(RobotContainer.CORAL_INTAKE::atTargetAngle),
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.EJECT)
        );
    }
}
