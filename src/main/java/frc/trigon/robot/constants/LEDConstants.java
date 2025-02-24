package frc.trigon.robot.constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import edu.wpi.first.wpilibj.util.Color;
import org.trigon.hardware.misc.leds.CANdleLEDStrip;
import org.trigon.hardware.misc.leds.LEDStrip;
import org.trigon.hardware.misc.leds.LEDStripAnimationSettings;

public class LEDConstants {
    public static final LEDStripAnimationSettings.ColorFlowSettings
            DEFAULT_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(Color.kDarkGreen, 0.1, false),
            MANUAL_GROUND_INTAKE_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(Color.kDarkMagenta, 0.5, false),
            GROUND_INTAKE_WITHOUT_CORAL_VISIBLE_TO_CAMERA_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(Color.kRed, 0.3, false),
            GROUND_INTAKE_WITH_CORAL_VISIBLE_TO_CAMERA_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(Color.kGreen, 0.6, false),
            CORAL_STATION_INTAKE_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(Color.kYellow, 0.4, true);
    public static final LEDStripAnimationSettings.BreatheSettings
            SCORE_CORAL_SETTINGS = new LEDStripAnimationSettings.BreatheSettings(Color.kGreen, 5, 0.2, false, LarsonAnimation.BounceMode.Back),
            SCORE_CORAL_AUTONOMOUSLY_SETTINGS = new LEDStripAnimationSettings.BreatheSettings(Color.kNavy, 7, 0.4, false, LarsonAnimation.BounceMode.Center);
    public static final LEDStripAnimationSettings.BlinkSettings
            INTAKE_CONFIRMATION_SETTINGS = new LEDStripAnimationSettings.BlinkSettings(Color.kDarkMagenta, 1),
            RELEASE_CORAL_SETTINGS = new LEDStripAnimationSettings.BlinkSettings(Color.kGreen, 1);
    public static final LEDStripAnimationSettings.StaticColorSettings
            

    private static final int CANDLE_ID = 0;
    static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);

    private static final int
            RIGHT_LED_STRIP_NUMBER_OF_LEDS = 17,
            LEFT_LED_STRIP_NUMBER_OF_LEDS = 17;
    private static final int LED_OFFSET = 8;
    private static final boolean
            RIGHT_LED_STRIP_INVERTED = false,
            LEFT_LED_STRIP_INVERTED = false;
    public static final LEDStrip
            RIGHT_LED_STRIP = LEDStrip.createCANdleLEDStrip(RIGHT_LED_STRIP_INVERTED, RIGHT_LED_STRIP_NUMBER_OF_LEDS, LED_OFFSET),
            LEFT_LED_STRIP = LEDStrip.createCANdleLEDStrip(LEFT_LED_STRIP_INVERTED, LEFT_LED_STRIP_NUMBER_OF_LEDS, RIGHT_LED_STRIP_NUMBER_OF_LEDS + LED_OFFSET);

    public static final double
            SCORING_BLINKING_SPEED = 0.5,
            INTAKE_GROUND_CORAL_BREATHING_SPEED = 0.3;
    public static final int SCORING_BREATHING_LEDS_AMOUNT = 5;
    public static final int RELEASE_CORAL_TIMEOUT_SECONDS = 1;

    /**
     * Initializes LEDConstants. Needed to be called for the LED strips to be initialized before being used.
     */
    public static void init() {
        final CANdleConfiguration config = new CANdleConfiguration();

        config.stripType = CANdle.LEDStripType.GRB;
        config.brightnessScalar = 0.5;
        config.disableWhenLOS = false;
        config.enableOptimizations = false;
        config.v5Enabled = true;
        config.statusLedOffWhenActive = true;

        CANDLE.configAllSettings(config);
        CANdleLEDStrip.setCANdle(CANDLE);
        CANdleLEDStrip.setTotalAmountOfLEDs(LED_OFFSET + RIGHT_LED_STRIP_NUMBER_OF_LEDS + LEFT_LED_STRIP_NUMBER_OF_LEDS);
    }
}