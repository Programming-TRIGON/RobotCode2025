package frc.trigon.robot.commands.commandclasses;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class ReefAlignmentCommand {
    public static Command alignToReef(int reefSideInAnalogClock, FieldConstants.ReefDirection reefDirection) {
        return switch (reefSideInAnalogClock) {
            case 6 -> alignToReefTo6oclock(reefDirection);
            case 8 -> alignToReefTo8oclock(reefDirection);
            case 10 -> alignToReefTo10oclock(reefDirection);
            case 12 -> alignToReefTo12oclock(reefDirection);
            case 2 -> alignToReefTo2oclock(reefDirection);
            case 4 -> alignToReefTo4oclock(reefDirection);
            default -> null;
        };
    }

    private static Command alignToReefTo6oclock(FieldConstants.ReefDirection reefDirection) {
        if (reefDirection == FieldConstants.ReefDirection.RIGHT) {
            return SwerveCommands.getDriveToPoseCommand(
                    () -> FieldConstants.REEF_6_OCLOCK_RIGHT_POSE,
                    new PathConstraints(1, 1, 1, 1)
            );
        }
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_6_OCLOCK_LEFT_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo8oclock(FieldConstants.ReefDirection reefDirection) {
        if (reefDirection == FieldConstants.ReefDirection.RIGHT) {
            return SwerveCommands.getDriveToPoseCommand(
                    () -> FieldConstants.REEF_8_OCLOCK_RIGHT_POSE,
                    new PathConstraints(1, 1, 1, 1)
            );
        }
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_8_OCLOCK_LEFT_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo10oclock(FieldConstants.ReefDirection reefDirection) {
        if (reefDirection == FieldConstants.ReefDirection.RIGHT) {
            return SwerveCommands.getDriveToPoseCommand(
                    () -> FieldConstants.REEF_10_OCLOCK_RIGHT_POSE,
                    new PathConstraints(1, 1, 1, 1)
            );
        }
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_10_OCLOCK_LEFT_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo12oclock(FieldConstants.ReefDirection reefDirection) {
        if (reefDirection == FieldConstants.ReefDirection.RIGHT) {
            return SwerveCommands.getDriveToPoseCommand(
                    () -> FieldConstants.REEF_12_OCLOCK_RIGHT_POSE,
                    new PathConstraints(1, 1, 1, 1)
            );
        }
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_12_OCLOCK_LEFT_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo2oclock(FieldConstants.ReefDirection reefDirection) {
        if (reefDirection == FieldConstants.ReefDirection.RIGHT) {
            return SwerveCommands.getDriveToPoseCommand(
                    () -> FieldConstants.REEF_2_OCLOCK_RIGHT_POSE,
                    new PathConstraints(1, 1, 1, 1)
            );
        }
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_2_OCLOCK_LEFT_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }

    private static Command alignToReefTo4oclock(FieldConstants.ReefDirection reefDirection) {
        if (reefDirection == FieldConstants.ReefDirection.RIGHT) {
            return SwerveCommands.getDriveToPoseCommand(
                    () -> FieldConstants.REEF_2_OCLOCK_RIGHT_POSE,
                    new PathConstraints(1, 1, 1, 1)
            );
        }
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.REEF_2_OCLOCK_LEFT_POSE,
                new PathConstraints(1, 1, 1, 1)
        );
    }
}
