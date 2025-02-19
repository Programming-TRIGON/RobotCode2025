// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandfactories.*;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.coralintake.CoralIntake;
import frc.trigon.robot.subsystems.elevator.Elevator;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.trigon.utilities.flippable.Flippable;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            CameraConstants.LEFT_REEF_TAG_CAMERA,
            CameraConstants.RIGHT_REEF_TAG_CAMERA
    );
    public static final Swerve SWERVE = new Swerve();
    public static final CoralIntake CORAL_INTAKE = new CoralIntake();
    public static final Elevator ELEVATOR = new Elevator();
    public static final Gripper GRIPPER = new Gripper();

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        OperatorConstants.OPERATOR_CONTROLLER.a().whileTrue(new PrintCommand("jk"));
        initializeGeneralSystems();
        buildAutoChooser();
        configureBindings();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Initializes the general systems of the robot.
     * Some systems need to be initialized at the start of the robot code so that others can use their functions.
     * For example, the LEDConstants need to be initialized so that the other systems can use them.
     */
    private void initializeGeneralSystems() {
        Flippable.init();
        LEDConstants.init();
        PathPlannerConstants.init();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
        bindSetters();
    }

    private void bindDefaultCommands() {
//        SWERVE.setDefaultCommand(GeneralCommands.getFieldRelativeDriveCommand());
//        CORAL_INTAKE.setDefaultCommand(CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.REST));
//        ELEVATOR.setDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST));
//        GRIPPER.setDefaultCommand(GripperCommands.getDefaultCommand());
//        configureSysIdBindings(ELEVATOR);
        OperatorConstants.OPERATOR_CONTROLLER.t().whileTrue(ElevatorCommands.getDebuggingCommand());
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_ROTATION_MODE_TRIGGER.onTrue(GeneralCommands.getToggleRotationModeCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());

        OperatorConstants.DEBUGGING_TRIGGER.whileTrue(CommandConstants.WHEEL_RADIUS_CHARACTERIZATION_COMMAND);
        OperatorConstants.FLOOR_CORAL_COLLECTION_TRIGGER.whileTrue(CoralCollectionCommands.getFloorCoralCollectionCommand());
        OperatorConstants.FEEDER_CORAL_COLLECTION_TRIGGER.whileTrue(CoralCollectionCommands.getFeederCoralCollectionCommand());
        OperatorConstants.SCORE_CORAL_IN_REEF_TRIGGER.whileTrue(CoralPlacingCommands.getScoreInReefCommand());

        OperatorConstants.EJECT_CORAL_TRIGGER.whileTrue(EjectionCommands.getEjectCoralCommand());
        OperatorConstants.UNLOAD_CORAL_TRIGGER.whileTrue(CoralCollectionCommands.getUnloadCoralCommand());
        OperatorConstants.COLLECT_ALGAE_TRIGGER.whileTrue(AlgaeManipulationCommands.getCollectAlgaeFromReefCommand());
        OperatorConstants.FEEDER_CORAL_COLLECTION_WITH_GRIPPER.whileTrue(CoralCollectionCommands.getFeederCoralCollectionFromGripperCommand());
    }

    private void bindSetters() {
        OperatorConstants.ENABLE_AUTO_CORAL_INTAKE_TRIGGER.onTrue(CommandConstants.ENABLE_AUTO_CORAL_INTAKE_COMMAND);
        OperatorConstants.DISABLE_AUTO_CORAL_INTAKE_TRIGGER.onTrue(CommandConstants.DISABLE_AUTO_CORAL_INTAKE_COMMAND);
        OperatorConstants.ENABLE_AUTONOMOUS_REEF_SCORING_TRIGGER.onTrue(CommandConstants.ENABLE_AUTONOMOUS_REEF_SCORING_COMMAND);
        OperatorConstants.DISABLE_AUTONOMOUS_REEF_SCORING_TRIGGER.onTrue(CommandConstants.DISABLE_AUTONOMOUS_REEF_SCORING_COMMAND);

        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_GRIPPER_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_GRIPPER_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_CORAL_INTAKE_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_CORAL_INTAKE_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L2_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_LEVEL_L2_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L3_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_LEVEL_L3_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_LEVEL_L4_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_LEVEL_L4_COMMAND);

        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_2_OCLOCK_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_2_OCLOCK_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_4_OCLOCK_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_4_OCLOCK_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_6_OCLOCK_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_6_OCLOCK_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_8_OCLOCK_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_8_OCLOCK_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_10_OCLOCK_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_10_OCLOCK_COMMAND);
        OperatorConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_12_OCLOCK_TRIGGER.onTrue(CommandConstants.SET_TARGET_SCORING_REEF_CLOCK_POSITION_12_OCLOCK_COMMAND);

        OperatorConstants.SET_TARGET_REEF_SCORING_SIDE_LEFT_TRIGGER.onTrue(CommandConstants.SET_TARGET_REEF_SCORING_SIDE_LEFT_COMMAND);
        OperatorConstants.SET_TARGET_REEF_SCORING_SIDE_RIGHT_TRIGGER.onTrue(CommandConstants.SET_TARGET_REEF_SCORING_SIDE_RIGHT_COMMAND);
    }

    private void configureSysIdBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(Commands.idle(subsystem));
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}