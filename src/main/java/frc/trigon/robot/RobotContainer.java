// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandfactories.GeneralCommands;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.constants.SimulatedGamePieceConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.coralintake.CoralIntake;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.Elevator;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.gripper.GripperConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.trigon.utilities.flippable.Flippable;

import java.util.ArrayList;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();
    public static final Swerve SWERVE = new Swerve();
    public static final CoralIntake CORAL_INTAKE = new CoralIntake();
    public static final Elevator ELEVATOR = new Elevator();
    public static final Gripper GRIPPER = new Gripper();

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
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

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);
        CORAL_INTAKE.setDefaultCommand(CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.REST));
        ELEVATOR.setDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST));
        GRIPPER.setDefaultCommand(GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.REST));
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

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_ROTATION_MODE_TRIGGER.onTrue(GeneralCommands.getToggleRotationModeCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());

        OperatorConstants.DRIVER_CONTROLLER.leftBumper().whileTrue(CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.COLLECT));
        OperatorConstants.DRIVER_CONTROLLER.a().whileTrue(GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.COLLECT_FROM_FEEDER).alongWith(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.COLLECT_FROM_FEEDER)));
        OperatorConstants.DRIVER_CONTROLLER.x().whileTrue(CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.EJECT));
        OperatorConstants.OPERATOR_CONTROLLER.z().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_L4).alongWith(GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_L4)));
        OperatorConstants.OPERATOR_CONTROLLER.x().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_L3).alongWith(GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_L3_OR_L2)));
        OperatorConstants.OPERATOR_CONTROLLER.c().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_L2).alongWith(GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_L3_OR_L2)));
        OperatorConstants.OPERATOR_CONTROLLER.v().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_L1).alongWith(GripperCommands.getSetTargetStateCommand(GripperConstants.GripperState.SCORE_L1)));

        Logger.recordOutput("LOL", mapSimulatedGamePieceListToPoseArray(SimulatedGamePieceConstants.CORAL_SCORING_LOCATIONS));
    }

    private static Pose3d[] mapSimulatedGamePieceListToPoseArray(ArrayList<Pose3d> gamePieces) {
        final Pose3d[] poses = new Pose3d[gamePieces.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = gamePieces.get(i);
        return poses;
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