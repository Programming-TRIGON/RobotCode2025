package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.commandclasses.IntakeAssistCommand;
import frc.trigon.robot.commands.commandfactories.AlgaeManipulationCommands;
import frc.trigon.robot.misc.ReefChooser;
import org.trigon.hardware.misc.KeyboardController;
import org.trigon.hardware.misc.XboxController;

public class OperatorConstants {
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.07;
    private static final int
            DRIVER_CONTROLLER_PORT = 0,
            REEF_CHOOSER_PORT = 1;
    private static final int
            CONTROLLER_TRANSLATION_EXPONENT = 2,
            CONTROLLER_ROTATION_EXPONENT = 1;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, CONTROLLER_ROTATION_EXPONENT, CONTROLLER_TRANSLATION_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();
    public static final ReefChooser REEF_CHOOSER = new ReefChooser(REEF_CHOOSER_PORT);
    private static boolean
            IS_LEFT_SCORE_BUTTON_PRESSED = false,
            IS_RIGHT_SCORE_BUTTON_PRESSED = false;

    public static final double
            POV_DIVIDER = 2,
            ROTATION_STICK_SPEED_DIVIDER = 1;

    public static final double INTAKE_ASSIST_SCALAR = 0.8;
    public static final IntakeAssistCommand.AssistMode DEFAULT_INTAKE_ASSIST_MODE = IntakeAssistCommand.AssistMode.ALTERNATE_ASSIST;

    public static final Trigger
            LED_AUTO_SETUP_TRIGGER = OPERATOR_CONTROLLER.backtick(),
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            DEBUGGING_TRIGGER = OPERATOR_CONTROLLER.f2(),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down(),
            CONTINUE_TRIGGER = OPERATOR_CONTROLLER.k().or(DRIVER_CONTROLLER.rightStick().and(DRIVER_CONTROLLER.leftStick()));
    public static final Trigger
            RESET_AMP_ALIGNER_TRIGGER = OPERATOR_CONTROLLER.v(),
            FLOOR_CORAL_COLLECTION_TRIGGER = DRIVER_CONTROLLER.leftTrigger().or(OPERATOR_CONTROLLER.c()),
            FEEDER_CORAL_COLLECTION_TRIGGER = OPERATOR_CONTROLLER.f().or(DRIVER_CONTROLLER.start()),
            RIGHT_SCORE_TRIGGER = OPERATOR_CONTROLLER.m().or(createScoreTrigger(DRIVER_CONTROLLER.rightStick().and(() -> !AlgaeManipulationCommands.IS_HOLDING_ALGAE), true)),
            LEFT_SCORE_TRIGGER = OPERATOR_CONTROLLER.b().or(createScoreTrigger(DRIVER_CONTROLLER.leftStick().and(() -> !AlgaeManipulationCommands.IS_HOLDING_ALGAE), false)),
            EJECT_CORAL_TRIGGER = OPERATOR_CONTROLLER.e().or(DRIVER_CONTROLLER.x().and(() -> !AlgaeManipulationCommands.IS_HOLDING_ALGAE)),
            UNLOAD_CORAL_TRIGGER = OPERATOR_CONTROLLER.z().or(DRIVER_CONTROLLER.back()),
            COLLECT_ALGAE_FROM_REEF_TRIGGER = OPERATOR_CONTROLLER.a().or(DRIVER_CONTROLLER.rightBumper()),
            COLLECT_ALGAE_FROM_L3_OVERRIDE_TRIGGER = DRIVER_CONTROLLER.leftStick().or(DRIVER_CONTROLLER.rightStick()),
            COLLECT_ALGAE_FROM_LOLLIPOP_TRIGGER = OPERATOR_CONTROLLER.l().or(DRIVER_CONTROLLER.leftBumper()),
            SCORE_ALGAE_IN_NET_TRIGGER = OPERATOR_CONTROLLER.n().or(createScoreTrigger(DRIVER_CONTROLLER.rightStick().and(() -> AlgaeManipulationCommands.IS_HOLDING_ALGAE), true)),
            SCORE_ALGAE_IN_PROCESSOR_TRIGGER = OPERATOR_CONTROLLER.j().or(createScoreTrigger(DRIVER_CONTROLLER.leftStick().and(() -> AlgaeManipulationCommands.IS_HOLDING_ALGAE), false)),
            STOP_ALGAE_AUTO_ALIGN_OVERRIDE_TRIGGER = DRIVER_CONTROLLER.x().and(() -> AlgaeManipulationCommands.IS_HOLDING_ALGAE);
    public static final Trigger
            ENABLE_INTAKE_ASSIST_TRIGGER = OPERATOR_CONTROLLER.o(),
            DISABLE_INTAKE_ASSIST_TRIGGER = OPERATOR_CONTROLLER.p(),
            ENABLE_AUTONOMOUS_REEF_SCORING_TRIGGER = OPERATOR_CONTROLLER.u(),
            DISABLE_AUTONOMOUS_REEF_SCORING_TRIGGER = OPERATOR_CONTROLLER.i(),
            TOGGLE_SHOULD_KEEP_INTAKE_OPEN_TRIGGER = DRIVER_CONTROLLER.b();
    public static final Trigger
            SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_GRIPPER_TRIGGER = OPERATOR_CONTROLLER.numpadDecimal(),
            SET_TARGET_SCORING_REEF_LEVEL_L1_FROM_CORAL_INTAKE_TRIGGER = OPERATOR_CONTROLLER.numpad0().or(DRIVER_CONTROLLER.povDown()),
            SET_TARGET_SCORING_REEF_LEVEL_L2_TRIGGER = OPERATOR_CONTROLLER.numpad1().or(DRIVER_CONTROLLER.povRight()),
            SET_TARGET_SCORING_REEF_LEVEL_L3_TRIGGER = OPERATOR_CONTROLLER.numpad2().or(DRIVER_CONTROLLER.povLeft()),
            SET_TARGET_SCORING_REEF_LEVEL_L4_TRIGGER = OPERATOR_CONTROLLER.numpad3().or(DRIVER_CONTROLLER.povUp()),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_2_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad9(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_4_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad6(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_6_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad5(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_8_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad4(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_10_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad7(),
            SET_TARGET_SCORING_REEF_CLOCK_POSITION_12_OCLOCK_TRIGGER = OPERATOR_CONTROLLER.numpad8(),
            SET_TARGET_REEF_SCORING_SIDE_LEFT_TRIGGER = OPERATOR_CONTROLLER.left(),
            SET_TARGET_REEF_SCORING_SIDE_RIGHT_TRIGGER = OPERATOR_CONTROLLER.right();

    private static Trigger createScoreTrigger(Trigger button, boolean isRight) {
        if (isRight)
            return button
                    .and(() -> !IS_LEFT_SCORE_BUTTON_PRESSED)
                    .onTrue(new InstantCommand(() -> IS_RIGHT_SCORE_BUTTON_PRESSED = true))
                    .onFalse(new InstantCommand(() -> IS_RIGHT_SCORE_BUTTON_PRESSED = false));
        return button
                .and(() -> !IS_RIGHT_SCORE_BUTTON_PRESSED)
                .onTrue(new InstantCommand(() -> IS_LEFT_SCORE_BUTTON_PRESSED = true))
                .onFalse(new InstantCommand(() -> IS_LEFT_SCORE_BUTTON_PRESSED = false));
    }
}