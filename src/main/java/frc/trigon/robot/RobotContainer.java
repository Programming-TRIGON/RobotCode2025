// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandclasses.CoralAlignmentCommand;
import frc.trigon.robot.commands.commandclasses.LEDAutoSetupCommand;
import frc.trigon.robot.commands.commandfactories.*;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.PathPlannerConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.algaemanipulator.AlgaeManipulator;
import frc.trigon.robot.subsystems.algaemanipulator.AlgaeManipulatorCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntake;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeCommands;
import frc.trigon.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.trigon.robot.subsystems.elevator.Elevator;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.gripper.Gripper;
import frc.trigon.robot.subsystems.gripper.GripperCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.trigon.utilities.flippable.Flippable;

import java.util.List;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            CameraConstants.LEFT_REEF_TAG_CAMERA,
            CameraConstants.RIGHT_REEF_TAG_CAMERA
    );
    public static final Swerve SWERVE = new Swerve();
    public static final AlgaeManipulator ALGAE_MANIPULATOR = new AlgaeManipulator();
    public static final CoralIntake CORAL_INTAKE = new CoralIntake();
    public static final Elevator ELEVATOR = new Elevator();
    public static final Gripper GRIPPER = new Gripper();
    public static final LoggedPowerDistribution POWER_DISTRIBUTION = LoggedPowerDistribution.getInstance(1, PowerDistribution.ModuleType.kRev);

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        initializeGeneralSystems();
        buildAutoChooser();
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
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
        PathPlannerConstants.init();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
        bindSetters();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(GeneralCommands.getFieldRelativeDriveCommand());
        ALGAE_MANIPULATOR.setDefaultCommand(AlgaeManipulatorCommands.getDefaultCommand());
        CORAL_INTAKE.setDefaultCommand(CoralIntakeCommands.getSetTargetStateCommand(CoralIntakeConstants.CoralIntakeState.REST));
        ELEVATOR.setDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.REST));
        GRIPPER.setDefaultCommand(GripperCommands.getGripperDefaultCommand());
    }

    private void bindControllerCommands() {
        OperatorConstants.LED_AUTO_SETUP_TRIGGER.onTrue(new LEDAutoSetupCommand(() -> autoChooser.get().getName()));
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());
        OperatorConstants.DEBUGGING_TRIGGER.whileTrue(CommandConstants.WHEEL_RADIUS_CHARACTERIZATION_COMMAND);

        OperatorConstants.OPERATOR_CONTROLLER.f3().whileTrue(new CoralAlignmentCommand());

        OperatorConstants.RESET_AMP_ALIGNER_TRIGGER.whileTrue(AlgaeManipulationCommands.getResetAmpAlignerCommand());

        OperatorConstants.FLOOR_CORAL_COLLECTION_TRIGGER.whileTrue(CoralCollectionCommands.getFloorCoralCollectionCommand());
//        OperatorConstants.FLOOR_CORAL_COLLECTION_TRIGGER.and(OperatorConstants.LEFT_MULTIFUNCTION_TRIGGER).whileTrue(new CoralAutoDriveCommand());
        OperatorConstants.FEEDER_CORAL_COLLECTION_TRIGGER.whileTrue(CoralCollectionCommands.getFeederCoralCollectionCommand());
        OperatorConstants.RIGHT_SCORE_TRIGGER.whileTrue(CoralPlacingCommands.getScoreInReefCommand(true));
        OperatorConstants.LEFT_SCORE_TRIGGER.whileTrue(CoralPlacingCommands.getScoreInReefCommand(false));
        OperatorConstants.EJECT_CORAL_TRIGGER.whileTrue(EjectionCommands.getEjectCoralCommand());
        OperatorConstants.UNLOAD_CORAL_TRIGGER.whileTrue(CoralCollectionCommands.getUnloadCoralCommand());

        OperatorConstants.COLLECT_ALGAE_FROM_REEF_TRIGGER.toggleOnTrue(AlgaeManipulationCommands.getCollectAlgaeFromReefCommand());
        OperatorConstants.COLLECT_ALGAE_FROM_LOLLIPOP_TRIGGER.toggleOnTrue(AlgaeManipulationCommands.getCollectAlgaeFromLollipopCommand());
        OperatorConstants.COLLECT_ALGAE_FROM_FLOOR_TRIGGER.toggleOnTrue(AlgaeManipulationCommands.getCollectAlgaeFromFloorCommand());
        OperatorConstants.ROLL_ALGAE_ON_FLOOR_TRIGGER.toggleOnTrue(AlgaeManipulationCommands.getRollAlgaeOnFloorCommand());
    }

    private void bindSetters() {
        OperatorConstants.ENABLE_IGNORE_LOLLIPOP_CORAL_TRIGGER.onTrue(CommandConstants.ENABLE_IGNORE_LOLLIPOP_CORAL_COMMAND);
        OperatorConstants.DISABLE_IGNORE_LOLLIPOP_CORAL_TRIGGER.onTrue(CommandConstants.DISABLE_IGNORE_LOLLIPOP_CORAL_COMMAND);
        OperatorConstants.ENABLE_AUTONOMOUS_REEF_SCORING_TRIGGER.onTrue(CommandConstants.ENABLE_AUTONOMOUS_REEF_SCORING_COMMAND);
        OperatorConstants.DISABLE_AUTONOMOUS_REEF_SCORING_TRIGGER.onTrue(CommandConstants.DISABLE_AUTONOMOUS_REEF_SCORING_COMMAND);
    }

    private void configureSysIdBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(Commands.idle(subsystem));
    }

    @SuppressWarnings("All")
    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser");

        final List<String> autoNames = AutoBuilder.getAllAutoNames();
        boolean hasDefault = false;

        for (String autoName : autoNames) {
            final PathPlannerAuto autoNonMirrored = new PathPlannerAuto(autoName);
            final PathPlannerAuto autoMirrored = new PathPlannerAuto(autoName, true);

            if (!PathPlannerConstants.DEFAULT_AUTO_NAME.isEmpty() && PathPlannerConstants.DEFAULT_AUTO_NAME.equals(autoName)) {
                hasDefault = true;
                autoChooser.addDefaultOption(autoNonMirrored.getName(), autoNonMirrored);
                autoChooser.addOption(autoMirrored.getName() + "Mirrored", autoMirrored);
            } else if (!PathPlannerConstants.DEFAULT_AUTO_NAME.isEmpty() && PathPlannerConstants.DEFAULT_AUTO_NAME.equals(autoName + "Mirrored")) {
                hasDefault = true;
                autoChooser.addDefaultOption(autoMirrored.getName() + "Mirrored", autoMirrored);
                autoChooser.addOption(autoNonMirrored.getName(), autoNonMirrored);
            } else {
                autoChooser.addOption(autoNonMirrored.getName(), autoNonMirrored);
                autoChooser.addOption(autoMirrored.getName() + "Mirrored", autoMirrored);
            }
        }

        if (!hasDefault)
            autoChooser.addDefaultOption("None", Commands.none());
        else
            autoChooser.addOption("None", Commands.none());

        addCommandsToChooser(
                AutonomousCommands.getFloorAutonomousCommand(true),
                AutonomousCommands.getFloorAutonomousCommand(false),
                AutonomousCommands.getFloorAutonomousCommand(true, FieldConstants.ReefClockPosition.REEF_2_OCLOCK, FieldConstants.ReefClockPosition.REEF_4_OCLOCK),
                AutonomousCommands.getFloorAutonomousCommand(false, FieldConstants.ReefClockPosition.REEF_8_OCLOCK, FieldConstants.ReefClockPosition.REEF_10_OCLOCK),
                AutonomousCommands.getFloorAutonomousCommand(true, FieldConstants.ReefClockPosition.REEF_2_OCLOCK, FieldConstants.ReefClockPosition.REEF_4_OCLOCK, FieldConstants.ReefClockPosition.REEF_6_OCLOCK),
                AutonomousCommands.getFloorAutonomousCommand(false, FieldConstants.ReefClockPosition.REEF_8_OCLOCK, FieldConstants.ReefClockPosition.REEF_10_OCLOCK, FieldConstants.ReefClockPosition.REEF_6_OCLOCK)
        );
    }

    private void addCommandsToChooser(Command... commands) {
        for (Command command : commands)
            autoChooser.addOption(command.getName(), command);
    }
}