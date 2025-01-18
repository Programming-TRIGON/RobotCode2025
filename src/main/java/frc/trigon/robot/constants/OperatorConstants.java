package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.trigon.hardware.misc.KeyboardController;
import org.trigon.hardware.misc.XboxController;

public class OperatorConstants {
    private static final int
            DRIVER_CONTROLLER_PORT = 0;
    private static final int DRIVER_CONTROLLER_EXPONENT = 1;
    private static final double DRIVER_CONTROLLER_DEADBAND = 0.1;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final double
            POV_DIVIDER = 2,
            STICKS_SPEED_DIVIDER = 1;

    public static final Trigger
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            TOGGLE_ROTATION_MODE_TRIGGER = DRIVER_CONTROLLER.b(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down(),
            CONTINUE_SCORING_TRIGGER = OPERATOR_CONTROLLER.k();

    public static final Trigger
            TURN_ON_AUTONOMOUS_REEF_SCORING_TRIGGER = OPERATOR_CONTROLLER.u(),
            TURN_OFF_AUTONOMOUS_REEF_SCORING_TRIGGER = OPERATOR_CONTROLLER.i(),
            SET_SCORING_TARGET_SCORING_LEVEL_L1_TRIGGER = OPERATOR_CONTROLLER.numpad0(),
            SET_SCORING_TARGET_SCORING_LEVEL_L2_TRIGGER = OPERATOR_CONTROLLER.numpad1(),
            SET_SCORING_TARGET_SCORING_LEVEL_L3_TRIGGER = OPERATOR_CONTROLLER.numpad2(),
            SET_SCORING_TARGET_SCORING_LEVEL_L4_TRIGGER = OPERATOR_CONTROLLER.numpad3(),
            SET_TARGET_REEF_CLOCK_POSITION_8_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad4(),
            SET_TARGET_REEF_CLOCK_POSITION_6_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad5(),
            SET_TARGET_REEF_CLOCK_POSITION_4_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad6(),
            SET_TARGET_REEF_CLOCK_POSITION_2_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad9(),
            SET_TARGET_REEF_CLOCK_POSITION_0_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad8(),
            SET_TARGET_REEF_CLOCK_POSITION_10_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad7();
}