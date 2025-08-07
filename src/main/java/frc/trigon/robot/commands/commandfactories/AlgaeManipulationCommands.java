package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.WaitUntilChangeCommand;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.ReefChooser;
import frc.trigon.robot.subsystems.algaemanipulator.AlgaeManipulatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.flippable.FlippablePose2d;
import org.trigon.utilities.flippable.FlippableRotation2d;
import org.trigon.utilities.flippable.FlippableTranslation2d;

import java.util.Map;

public class AlgaeManipulationCommands {
    private static final ReefChooser REEF_CHOOSER = OperatorConstants.REEF_CHOOSER;
    public static boolean IS_HOLDING_ALGAE = false;

    public static Command getCollectAlgaeFromLollipopCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> IS_HOLDING_ALGAE = true),
                CoralCollectionCommands.getUnloadCoralCommand().onlyIf(RobotContainer.GRIPPER::hasGamePiece).asProxy(),
                getGripAlgaeCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_LOLLIPOP).asProxy()
                        .until(AlgaeManipulationCommands::isScoreAlgaeButtonPressed),
                getScoreAlgaeCommand().asProxy()
        )
                .raceWith(getScoreNetEndingConditionCommand(), getScoreProcessorEndingConditionCommand())
                .andThen(AlgaeManipulationCommands::reloadAfterScore)
                .finallyDo(AlgaeManipulationCommands::disableIsHoldingAlgae);
    }

    public static Command getCollectAlgaeFromReefCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> IS_HOLDING_ALGAE = true),
                CoralCollectionCommands.getUnloadCoralCommand().onlyIf(RobotContainer.GRIPPER::hasGamePiece).asProxy(),
                getCollectAlgaeFromReefManuallyCommand().asProxy(),
                getScoreAlgaeCommand().asProxy()
        )
                .alongWith(getAlignToReefCommand().onlyIf(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY).asProxy())
                .raceWith(getScoreNetEndingConditionCommand(), getScoreProcessorEndingConditionCommand())
                .andThen(AlgaeManipulationCommands::reloadAfterScore)
                .finallyDo(AlgaeManipulationCommands::disableIsHoldingAlgae);
    }

    public static Command getResetAmpAlignerCommand() {
        return new ParallelRaceGroup(
                GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.REST),
                GeneralCommands.runWhen(AlgaeManipulatorCommands.getPressLimitSwitchCommand(), () -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.REST))
        );
    }

    private static void reloadAfterScore() {
        new WaitUntilCommand(() -> RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.REST)).andThen(CoralCollectionCommands.getLoadCoralCommand().asProxy()).onlyIf(() -> RobotContainer.CORAL_INTAKE.hasGamePiece() && REEF_CHOOSER.getScoringLevel() != CoralPlacingCommands.ScoringLevel.L1_CORAL_INTAKE).schedule();
    }

    private static void disableIsHoldingAlgae() {
        new WaitUntilCommand(() -> RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.REST)).andThen(() -> IS_HOLDING_ALGAE = false).schedule();
    }

    private static Command getScoreNetEndingConditionCommand() {
        return new WaitUntilCommand(() -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.SCORE_ALGAE_IN_NET)).andThen(new WaitCommand(0.1));
    }

    private static Command getScoreProcessorEndingConditionCommand() {
        return new WaitUntilCommand(() -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.SCORE_ALGAE_IN_PROCESSOR)).andThen(new WaitCommand(0.1)).andThen(new WaitUntilCommand(() -> !OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean()));
    }

    private static Command getCollectAlgaeFromReefManuallyCommand() {
        return new ParallelCommandGroup(
                getGripAlgaeCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_REEF),
                getOpenElevatorForAlgaeCommand()
        ).raceWith(
                new SequentialCommandGroup(
                        new WaitCommand(1),
                        new WaitUntilCommand(RobotContainer.GRIPPER::isMovingSlowly),
                        new WaitUntilCommand(() -> !OperatorConstants.COLLECT_ALGAE_FROM_L3_OVERRIDE_TRIGGER.getAsBoolean())
                )
        );
    }

    private static Command getGripAlgaeCommand(GripperConstants.GripperState targetGripState) {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateWhileHoldingAlgaeCommand(targetGripState).raceWith(new WaitCommand(1).andThen(new WaitUntilCommand(RobotContainer.GRIPPER::isMovingSlowly))),
                GripperCommands.getSetTargetStateWhileHoldingAlgaeCommand(GripperConstants.GripperState.HOLD_ALGAE)
        ).alongWith(
                AlgaeManipulatorCommands.getOpenCommand()
        );
    }

    private static Command getOpenElevatorForAlgaeCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.COLLECT_ALGAE_FROM_L3),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST_WITH_ALGAE),
                OperatorConstants.COLLECT_ALGAE_FROM_L3_OVERRIDE_TRIGGER
        );
    }

    private static Command getScoreAlgaeCommand() {
        return new SelectCommand<>(
                Map.of(
                        0, getHoldAlgaeCommand(),
                        1, getScoreInNetCommand(),
                        2, getScoreInProcessorCommand()
                ),
                AlgaeManipulationCommands::getAlgaeScoreMethodSelector
        ).raceWith(new WaitUntilChangeCommand<>(AlgaeManipulationCommands::isScoreAlgaeButtonPressed)).repeatedly();
    }

    private static int getAlgaeScoreMethodSelector() {
        if (OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean())
            return 1;
        else if (OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean())
            return 2;
        return 0;
    }

    private static Command getHoldAlgaeCommand() {
        return new ParallelCommandGroup(
                GripperCommands.getSetTargetStateWhileHoldingAlgaeCommand(GripperConstants.GripperState.HOLD_ALGAE),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST_WITH_ALGAE),
                AlgaeManipulatorCommands.getOpenCommand()
        );
    }

    private static Command getScoreInNetCommand() {
        return new ParallelRaceGroup(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_NET),
                AlgaeManipulatorCommands.getOpenCommand(),
                new SequentialCommandGroup(
                        GripperCommands.getSetTargetStateWhileHoldingAlgaeCommand(GripperConstants.GripperState.PREPARE_FOR_SCORING_ALGAE_IN_NET).until(OperatorConstants.CONTINUE_TRIGGER),
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_ALGAE_IN_NET).withTimeout(0.5)
                ),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(Rotation2d.k180deg, true)
                ).asProxy().onlyWhile(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY)
        );
    }

    private static Command getScoreInProcessorCommand() {
        return new ParallelCommandGroup(
                AlgaeManipulatorCommands.getOpenCommand(),
                new SequentialCommandGroup(
                        GripperCommands.getSetTargetStateWhileHoldingAlgaeCommand(GripperConstants.GripperState.PREPARE_FOR_SCORING_ALGAE_IN_PROCESSOR).until(OperatorConstants.CONTINUE_TRIGGER),
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_ALGAE_IN_PROCESSOR)
                ),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(Rotation2d.fromDegrees(90), true)
                ).asProxy().onlyWhile(() -> CoralPlacingCommands.SHOULD_SCORE_AUTONOMOUSLY)
        ).until(() -> RobotContainer.GRIPPER.atState(GripperConstants.GripperState.SCORE_ALGAE_IN_PROCESSOR) && !OperatorConstants.CONTINUE_TRIGGER.getAsBoolean());
    }

    private static Command getAlignToReefCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        AlgaeManipulationCommands::calculateClosestAlgaeCollectionPose,
                        AutonomousConstants.DRIVE_TO_REEF_CONSTRAINTS
                ),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> 0,
                        () -> calculateClosestAlgaeCollectionPose().getRotation()
                )
        ).raceWith(
                new WaitCommand(1).andThen(new WaitUntilCommand(RobotContainer.GRIPPER::isMovingSlowly)),
                new WaitUntilCommand(OperatorConstants.STOP_ALGAE_AUTO_ALIGN_OVERRIDE_TRIGGER)
        );
    }

    private static boolean isScoreAlgaeButtonPressed() {
        return OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean() ||
                OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean();
    }

    private static FlippablePose2d calculateClosestAlgaeCollectionPose() {
        final Translation2d robotPositionOnField = RobotContainer.POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation();
        final Translation2d reefCenterPosition = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d[] reefClockAngles = FieldConstants.REEF_CLOCK_ANGLES;
        final Transform2d reefCenterToBranchScoringPose = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_ALGAE_COLLECTION_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d());

        double distanceFromClosestScoringPoseMeters = Double.POSITIVE_INFINITY;
        Pose2d closestScoringPose = new Pose2d();
        for (final Rotation2d targetRotation : reefClockAngles) {
            final Pose2d reefCenterAtTargetRotation = new Pose2d(reefCenterPosition, targetRotation);
            final Pose2d currentScoringPose = reefCenterAtTargetRotation.transformBy(reefCenterToBranchScoringPose);
            final double distanceFromCurrentScoringPoseMeters = currentScoringPose.getTranslation().getDistance(robotPositionOnField);
            if (distanceFromCurrentScoringPoseMeters < distanceFromClosestScoringPoseMeters) {
                distanceFromClosestScoringPoseMeters = distanceFromCurrentScoringPoseMeters;
                closestScoringPose = currentScoringPose;
            }
        }

        return new FlippablePose2d(closestScoringPose, false);
    }

    private static double fieldRelativePowersToSelfRelativeXPower(double xPower, double yPower) {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();
        final double xValue = CommandConstants.calculateDriveStickAxisValue(xPower);
        final double yValue = CommandConstants.calculateDriveStickAxisValue(yPower);

        return (xValue * robotHeading.getCos()) + (yValue * robotHeading.getSin());
    }
}