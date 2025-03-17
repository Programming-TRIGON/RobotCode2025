package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.algaemanipulator.AlgaeManipulatorCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.flippable.FlippableRotation2d;

public class AlgaeManipulationCommands {
    public static Command getCollectAlgaeFromFloorCommand() {
        return new SequentialCommandGroup(
                CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT_ALGAE_FROM_FLOOR)
        );
    }

    public static Command getCollectAlgaeFromLollipopCommand() {
        return getGripAlgaeCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_LOLLIPOP)
                .until(OperatorConstants.SCORE_IN_NET_TRIGGER).andThen(
                        getScoreInNetCommand()
                );
    }

    public static Command getCollectAlgaeFromReefCommand() {
        return new ParallelCommandGroup(
                getOpenElevatorForAlgaeCommand(),
                getGripAlgaeCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_REEF)
        ).until(OperatorConstants.SCORE_IN_NET_TRIGGER).andThen(
                getScoreInNetCommand()
        );
    }

    private static Command getOpenElevatorForAlgaeCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.COLLECT_ALGAE_FROM_L3),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST_WITH_ALGAE),
                OperatorConstants.MULTIFUNCTION_TRIGGER
        );
    }

    private static Command getGripAlgaeCommand(GripperConstants.GripperState targetGripperGripState) {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateWithCurrentCommand(targetGripperGripState).raceWith(new WaitCommand(1).andThen(new WaitUntilCommand(RobotContainer.GRIPPER::isMovingSlowly))),
                GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.HOLD_ALGAE)
        ).alongWith(
                AlgaeManipulatorCommands.getOpenCommand()
        );
    }

    private static Command getScoreInNetCommand() {
        return new ParallelCommandGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_NET),
                AlgaeManipulatorCommands.getOpenCommand(),
                new SequentialCommandGroup(
                        GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.PREPARE_FOR_SCORING_ALGAE_IN_NET).until(OperatorConstants.CONTINUE_TRIGGER),
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_ALGAE_IN_NET)
                ),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(Rotation2d.k180deg, true)
                ).asProxy()
        );
    }

    private static Command getGripAlgaeCommand() {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_REEF).raceWith(new WaitCommand(1).andThen(new WaitUntilCommand(RobotContainer.GRIPPER::isMovingSlowly))),
                GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.HOLD_ALGAE)
        ).alongWith(
                AlgaeManipulatorCommands.getOpenCommand()
        );
    }
}