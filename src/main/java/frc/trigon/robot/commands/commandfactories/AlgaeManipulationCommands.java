package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.WaitUntilChangeCommand;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
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

public class AlgaeManipulationCommands {
    public static boolean SHOULD_ALIGN_TO_REEF = true;
    private static final ReefChooser REEF_CHOOSER = OperatorConstants.REEF_CHOOSER;

    public static Command getCollectAlgaeFromLollipopCommand() {
        return new SequentialCommandGroup(
                CoralCollectionCommands.getUnloadCoralCommand().asProxy(),
                getGripAlgaeCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_LOLLIPOP).asProxy().until(() -> OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean() || OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean()),
                getScoreAlgaeCommand().asProxy()
        ).raceWith(getScoreNetEndingConditionCommand()).andThen(AlgaeManipulationCommands::reloadAfterScore);
    }

    public static Command getCollectAlgaeFromReefCommand() {
        return CoralCollectionCommands.getUnloadCoralCommand().asProxy().andThen(
                new ParallelCommandGroup(
                        getCollectAlgaeFromReefManuallyCommand().asProxy(),
                        getAlignToReefCommand().onlyIf(() -> SHOULD_ALIGN_TO_REEF && !OperatorConstants.RIGHT_MULTIFUNCTION_TRIGGER.getAsBoolean()).asProxy()
                ).raceWith(
                        new WaitUntilChangeCommand<>(REEF_CHOOSER::getClockPosition)
                ).repeatedly()
        ).raceWith(getScoreNetEndingConditionCommand()).andThen(AlgaeManipulationCommands::reloadAfterScore);
    }

    private static void reloadAfterScore() {
        new WaitUntilCommand(() -> RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.REST)).andThen(CoralCollectionCommands.getLoadCoralCommand().asProxy()).onlyIf(() -> RobotContainer.CORAL_INTAKE.hasGamePiece() && REEF_CHOOSER.getScoringLevel() != CoralPlacingCommands.ScoringLevel.L1_CORAL_INTAKE).schedule();
    }

    private static Command getScoreNetEndingConditionCommand() {
        return new WaitUntilCommand(() -> OperatorConstants.CONTINUE_TRIGGER.getAsBoolean() && RobotContainer.ELEVATOR.atState(ElevatorConstants.ElevatorState.SCORE_NET)).andThen(new WaitCommand(0.5));
    }

    private static Command getCollectAlgaeFromReefManuallyCommand() {
        return new ParallelCommandGroup(
                getGripAlgaeCommand(GripperConstants.GripperState.COLLECT_ALGAE_FROM_REEF),
                getOpenElevatorForAlgaeCommand()
        ).until(() -> OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER.getAsBoolean() || OperatorConstants.SCORE_ALGAE_IN_PROCESSOR_TRIGGER.getAsBoolean()).andThen(
                getScoreAlgaeCommand()
        );
    }

    private static Command getGripAlgaeCommand(GripperConstants.GripperState targetGripState) {
        return new SequentialCommandGroup(
                GripperCommands.getSetTargetStateWithCurrentCommand(targetGripState).raceWith(new WaitCommand(1).andThen(new WaitUntilCommand(RobotContainer.GRIPPER::isMovingSlowly))),
                GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.HOLD_ALGAE)
        ).alongWith(
                AlgaeManipulatorCommands.getOpenCommand()
        );
    }

    private static Command getOpenElevatorForAlgaeCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.COLLECT_ALGAE_FROM_L3),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST_WITH_ALGAE),
                OperatorConstants.LEFT_MULTIFUNCTION_TRIGGER
        );
    }

    private static Command getScoreAlgaeCommand() {
        return new ConditionalCommand(
                getScoreInNetCommand(),
                getScoreInProcessorCommand(),
                OperatorConstants.SCORE_ALGAE_IN_NET_TRIGGER
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

    private static Command getScoreInProcessorCommand() {
        return new ParallelCommandGroup(
                AlgaeManipulatorCommands.getOpenCommand().until(OperatorConstants.CONTINUE_TRIGGER),
                new SequentialCommandGroup(
                        GripperCommands.getSetTargetStateWithCurrentCommand(GripperConstants.GripperState.PREPARE_FOR_SCORING_ALGAE_IN_PROCESSOR).until(OperatorConstants.CONTINUE_TRIGGER),
                        GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_ALGAE_IN_PROCESSOR)
                ),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(

                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> new FlippableRotation2d(Rotation2d.fromDegrees(90), true)
                ).asProxy()
        );
    }

    private static Command getAlignToReefCommand() {
        return new SequentialCommandGroup(
                SwerveCommands.getDriveToPoseCommand(
                        AlgaeManipulationCommands::calculateClosestAlgaeCollectionPose,
                        PathPlannerConstants.DRIVE_TO_REEF_CONSTRAINTS
                ),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> 0,
                        () -> calculateClosestAlgaeCollectionPose().getRotation()
                )
        ).until(OperatorConstants.CONTINUE_TRIGGER);
    }

    private static Pose2d calculateReefAlgaeCollectionPose() {
        final Translation2d reefCenterPositionOnField = new FlippableTranslation2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, true).get();
        final Rotation2d targetAngle = calculateTargetAngle().get();
        final Transform2d reefCenterToTargetPoseTranslation = new Transform2d(FieldConstants.REEF_CENTER_TO_TARGET_ALGAE_COLLECTION_POSITION_X_TRANSFORM_METERS, 0, new Rotation2d());

        return new Pose2d(reefCenterPositionOnField, targetAngle).transformBy(reefCenterToTargetPoseTranslation);
    }

    private static FlippableRotation2d calculateTargetAngle() {
        return new FlippableRotation2d(REEF_CHOOSER.getClockPosition().clockAngle, true);
    }

    public static FlippablePose2d calculateClosestAlgaeCollectionPose() {
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