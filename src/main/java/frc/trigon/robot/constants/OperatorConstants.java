package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.misc.ReefChooser;
import org.trigon.hardware.misc.KeyboardController;
import org.trigon.hardware.misc.XboxController;

public class OperatorConstants {
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.07;
    private static final int
            DRIVER_CONTROLLER_PORT = 0,
            REEF_CHOOSER_PORT = 1;
    private static final int DRIVER_CONTROLLER_EXPONENT = 2;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();
    public static final ReefChooser REEF_CHOOSER = new ReefChooser(REEF_CHOOSER_PORT);

    public static final double
            POV_DIVIDER = 2,
            ROTATION_STICK_SPEED_DIVIDER = 1;

    public static final Trigger
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            TOGGLE_ROTATION_MODE_TRIGGER = DRIVER_CONTROLLER.b().or(DRIVER_CONTROLLER.start()),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            MULTIFUNCTION_TRIGGER = DRIVER_CONTROLLER.leftStick().or(OPERATOR_CONTROLLER.m()),
            OVERRIDE_AUTO_SCORING_TO_CLOSEST_SCORING_LOCATION_TRIGGER = DRIVER_CONTROLLER.rightStick(),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down(),
            FLOOR_CORAL_COLLECTION_TRIGGER = DRIVER_CONTROLLER.leftTrigger().or(OPERATOR_CONTROLLER.c()),
            FEEDER_CORAL_COLLECTION_TRIGGER = OPERATOR_CONTROLLER.f().or(DRIVER_CONTROLLER.start()),
            SCORE_CORAL_IN_REEF_TRIGGER = DRIVER_CONTROLLER.rightBumper().or(OPERATOR_CONTROLLER.q()),
            CONTINUE_TRIGGER = OPERATOR_CONTROLLER.k().or(DRIVER_CONTROLLER.leftBumper()),
            EJECT_CORAL_TRIGGER = OPERATOR_CONTROLLER.e().or(DRIVER_CONTROLLER.x()),
            DEBUGGING_TRIGGER = OPERATOR_CONTROLLER.f2(),
            UNLOAD_CORAL_TRIGGER = OPERATOR_CONTROLLER.z().or(DRIVER_CONTROLLER.back()),
            COLLECT_ALGAE_TRIGGER = OPERATOR_CONTROLLER.a(),
            FEEDER_CORAL_COLLECTION_WITH_GRIPPER = OPERATOR_CONTROLLER.d(),
            SCORE_IN_NET_TRIGGER = OPERATOR_CONTROLLER.n(),
            LED_AUTO_SETUP_TRIGGER = OPERATOR_CONTROLLER.backtick();
    public static final Trigger
            ENABLE_AUTO_CORAL_INTAKE_TRIGGER = OPERATOR_CONTROLLER.o(),
            DISABLE_AUTO_CORAL_INTAKE_TRIGGER = OPERATOR_CONTROLLER.p(),
            ENABLE_AUTONOMOUS_REEF_SCORING_TRIGGER = OPERATOR_CONTROLLER.u(),
            DISABLE_AUTONOMOUS_REEF_SCORING_TRIGGER = OPERATOR_CONTROLLER.i(),
            SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_GRIPPER_TRIGGER = OPERATOR_CONTROLLER.numpadDecimal(),
            SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_CORAL_INTAKE_TRIGGER = OPERATOR_CONTROLLER.numpad0().or(() -> DRIVER_CONTROLLER.getPov() >= 270),
            SET_TARGET_SCORING_REEF_LEVEL_L2_TRIGGER = OPERATOR_CONTROLLER.numpad1().or(() -> DRIVER_CONTROLLER.getPov() >= 180 && DRIVER_CONTROLLER.getPov() < 270),
            SET_TARGET_SCORING_REEF_LEVEL_L3_TRIGGER = OPERATOR_CONTROLLER.numpad2().or(() -> DRIVER_CONTROLLER.getPov() >= 90 && DRIVER_CONTROLLER.getPov() < 180),
            SET_TARGET_SCORING_REEF_LEVEL_L4_TRIGGER = OPERATOR_CONTROLLER.numpad3().or(() -> DRIVER_CONTROLLER.getPov() >= 0 && DRIVER_CONTROLLER.getPov() < 90),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_2_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad9(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_4_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad6(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_6_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad5(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_8_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad4(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_10_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad7(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_12_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad8(),
            SET_TARGET_REEF_SCORING_SIDE_LEFT_TRIGGER = OPERATOR_CONTROLLER.left(),
            SET_TARGET_REEF_SCORING_SIDE_RIGHT_TRIGGER = OPERATOR_CONTROLLER.right();
}