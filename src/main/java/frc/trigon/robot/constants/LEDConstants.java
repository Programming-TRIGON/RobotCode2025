package frc.trigon.robot.constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import org.trigon.hardware.misc.leds.CANdleLEDStrip;
import org.trigon.hardware.misc.leds.LEDStrip;

import static org.trigon.hardware.misc.leds.LEDStrip.createCANdleLEDStrip;

public class LEDConstants {
    public static final double
            SCORING_BLINKING_SPEED = 0.5,
            RELEASE_CORAL_BREATHING_SPEED = 0.3,
            INTAKE_GROUND_CORAL_BREATHING_SPEED = 0.3;
    public static final int
            SCORING_BREATHING_LEDS_AMOUNT = 5;

    private static final int CANDLE_ID = 0;
    private static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);

    private static final boolean
            IS_RIGHT_STRIP_INVERTED = false,
            IS_LEFT_STRIP_INVERTED = false;
    private static final int
            RIGHT_STRIP_NUMBER_OF_LEDS = 20,
            LEFT_STRIP_NUMBER_OF_LEDS = 20;
    private static final int LED_OFFSET = 8;

    private static final LEDStrip[] LED_STRIPS = new LEDStrip[]{
            createCANdleLEDStrip(IS_RIGHT_STRIP_INVERTED, RIGHT_STRIP_NUMBER_OF_LEDS, LED_OFFSET),
            createCANdleLEDStrip(IS_LEFT_STRIP_INVERTED, LEFT_STRIP_NUMBER_OF_LEDS, LEFT_STRIP_NUMBER_OF_LEDS + LED_OFFSET)
    };


    /**
     * Initializes LEDConstants. Needed to be called for the LED strips to be initialized before being used.
     */
    public static void init() {
        final CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = CANdle.LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        config.disableWhenLOS = true;
        config.enableOptimizations = true;
        CANDLE.configAllSettings(config);

        CANdleLEDStrip.setCANdle(CANDLE);
        CANdleLEDStrip.setTotalAmountOfLEDs(25);
    }
}