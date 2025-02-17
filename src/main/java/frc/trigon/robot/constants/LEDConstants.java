package frc.trigon.robot.constants;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import org.trigon.hardware.misc.leds.LEDStrip;
import static org.trigon.hardware.misc.leds.LEDStrip.createCANdleLEDStrip;

public class LEDConstants {
    //TODO: Implement LEDConstants
    private static final int CANDLE_ID = 0;
    private static final CANdle.LEDStripType STRIP_TYPE = CANdle.LEDStripType.RGB;
    private static final double BRIGHTNESS_SCALAR = 0.5;
    static final double
        MINIMUM_BATERRY = 10.5,
        LOW_BATTERY_FLASHING_SPEED = 1;

    static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);

    private final boolean LED_STRIP_IS_INVERTED = true;
    private final int NUMBER_OF_LEDS = 25;
    private final int LED_OFFSET = 8;

    private final LEDStrip LED_STRIP = createCANdleLEDStrip(LED_STRIP_IS_INVERTED, NUMBER_OF_LEDS, LED_OFFSET);


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

    }
}