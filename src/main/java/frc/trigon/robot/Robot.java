// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.Phoenix6Inputs;

public class Robot extends LoggedRobot {
    public static final boolean IS_REAL = Robot.isReal();
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    Robot() {
        RobotConstants.init();
        configLogger();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        Phoenix6Inputs.refreshAllInputs();
        commandScheduler.run();
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void autonomousInit() {
        RobotContainer.SWERVE.initializeDrive(true);
        RobotContainer.POSE_ESTIMATOR.resetOdometry();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void testInit() {
        commandScheduler.cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        if (RobotHardwareStats.isSimulation())
            AprilTagCameraConstants.VISION_SIMULATION.update(RobotContainer.POSE_ESTIMATOR.getEstimatedOdometryPose());
        SimulationFieldHandler.update();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
        if (autonomousCommand.getName().contains("Floor"))
            return;

        if (!RobotContainer.SWERVE.isPathPlannerDriving) {
//            if (!RobotContainer.SWERVE.atPose(new FlippablePose2d(RobotContainer.SWERVE.targetPathPlannerPose, false)))
            RobotContainer.SWERVE.drivePathPlanner(new ChassisSpeeds(), false);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

    private void configLogger() {
        if (RobotHardwareStats.isReplay()) {
            setUseTiming(false);
            final String logPath = LogFileUtil.findReplayLog();
            final String logWriterPath = LogFileUtil.addPathSuffix(logPath, "_replay");

            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(logWriterPath));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter(RobotConstants.LOGGING_PATH));
        }

        Logger.start();
        SignalLogger.enableAutoLogging(false);
    }
}
