package frc.trigon.robot.constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;
import org.trigon.hardware.misc.leds.CANdleLEDStrip;
import org.trigon.hardware.misc.leds.LEDStrip;
import org.trigon.hardware.misc.leds.LEDStripAnimationSettings;

public class LEDConstants {
    public static final LEDStripAnimationSettings.LEDAnimationSettings
            DEFAULT_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(new Color(0, 150 / 255.0, 1), 0.2, false),
            GROUND_INTAKE_WITHOUT_CORAL_VISIBLE_TO_CAMERA_SETTINGS = new LEDStripAnimationSettings.StaticColorSettings(Color.kRed),
            GROUND_INTAKE_WITH_CORAL_VISIBLE_TO_CAMERA_SETTINGS = new LEDStripAnimationSettings.StaticColorSettings(Color.kGreen),
            CORAL_STATION_INTAKE_SETTINGS = new LEDStripAnimationSettings.ColorFlowSettings(Color.kYellow, 0.4, true),
            INTAKE_CONFIRMATION_SETTINGS = new LEDStripAnimationSettings.BlinkSettings(Color.kDarkMagenta, 1),
            RELEASE_CORAL_SETTINGS = new LEDStripAnimationSettings.BlinkSettings(Color.kGreen, 1);

    private static final int CANDLE_ID = 0;
    static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);

    private static final int
            RIGHT_LED_STRIP_NUMBER_OF_LEDS = 19,
            LEFT_LED_STRIP_NUMBER_OF_LEDS = 19;
    private static final int LED_OFFSET = 8;
    private static final boolean
            RIGHT_LED_STRIP_INVERTED = false,
            LEFT_LED_STRIP_INVERTED = false;
    public static final LEDStrip
            RIGHT_LED_STRIP = LEDStrip.createCANdleLEDStrip(RIGHT_LED_STRIP_INVERTED, RIGHT_LED_STRIP_NUMBER_OF_LEDS, LED_OFFSET),
            LEFT_LED_STRIP = LEDStrip.createCANdleLEDStrip(LEFT_LED_STRIP_INVERTED, LEFT_LED_STRIP_NUMBER_OF_LEDS, RIGHT_LED_STRIP_NUMBER_OF_LEDS + LED_OFFSET);

    public static final double RELEASE_CORAL_TIMEOUT_SECONDS = 0.5;

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