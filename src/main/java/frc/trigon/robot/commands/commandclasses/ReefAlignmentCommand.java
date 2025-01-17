package frc.trigon.robot.commands.commandclasses;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class ReefAlignmentCommand {
    public static Command alignToReef(int reefSideInAnalogClock) {
        return switch (reefSideInAnalogClock) {
            case 6 -> alignToReefTo6oclock();
            case 8 -> alignToReefTo8oclock();
            case 10 -> alignToReefTo10oclock();
            case 12 -> alignToReefTo12oclock();
            case 2 -> alignToReefTo2oclock();
            case 4 -> alignToReefTo4oclock();
            default -> null;
        };
    }

    private static Command alignToReefTo6oclock() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_6_OCLOCK_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo8oclock() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_8_OCLOCK_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo10oclock() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_10_OCLOCK_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo12oclock() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_12_OCLOCK_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo2oclock() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_2_OCLOCK_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo4oclock() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_4_OCLOCK_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }
}
