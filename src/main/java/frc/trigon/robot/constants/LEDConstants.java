package frc.trigon.robot.constants;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import org.trigon.hardware.misc.leds.CANdleLEDStrip;
import org.trigon.hardware.misc.leds.LEDStrip;

import static org.trigon.hardware.misc.leds.LEDStrip.createCANdleLEDStrip;

public class LEDConstants {
    private static final int CANDLE_ID = 0;
    private static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);
    private static final CANdle.LEDStripType STRIP_TYPE = CANdle.LEDStripType.RGB;
    private static final double BRIGHTNESS_SCALAR = 0.5;

    private static final boolean IS_LED_STRIP_INVERTED = true;
    private static final int NUMBER_OF_LEDS = 25;
    private static final int LED_OFFSET = 8;

    private static final LEDStrip LED_STRIP = createCANdleLEDStrip(IS_LED_STRIP_INVERTED, NUMBER_OF_LEDS, LED_OFFSET);


    /**
     * Initializes LEDConstants. Needed to be called for the LED strips to be initialized before being used.
     */
    public static void init() {
        final CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = STRIP_TYPE;
        config.brightnessScalar = BRIGHTNESS_SCALAR;
        config.disableWhenLOS = true;
        config.enableOptimizations = true;
        CANDLE.configAllSettings(config);

        CANdleLEDStrip.setCANdle(CANDLE);
        CANdleLEDStrip.setTotalAmountOfLEDs(25);
    }

    static final double
            MINIMUM_BATERRY_VOLTS = 10.5,
            LOW_BATTERY_FLASHING_SPEED = 0.5;
}