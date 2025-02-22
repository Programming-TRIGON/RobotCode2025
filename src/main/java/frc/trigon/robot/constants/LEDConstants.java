package frc.trigon.robot.constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import org.trigon.hardware.misc.leds.CANdleLEDStrip;
import org.trigon.hardware.misc.leds.LEDStrip;

public class LEDConstants {
    private static final int CANDLE_ID = 0;
    private static final CANdle.LEDStripType STRIP_TYPE = CANdle.LEDStripType.RGB;
    private static final double BRIGHTNESS_SCALAR = 0.5;
    static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);

    private static final int
            RIGHT_CLIMBER_NUMBER_OF_LEDS = 20,
            LEFT_CLIMBER_NUMBER_OF_LEDS = 20;
    private static final int LED_OFFSET = 8;
    private static final boolean
            RIGHT_CLIMBER_INVERTED = false,
            LEFT_CLIMBER_INVERTED = false;
    public static final LEDStrip
            RIGHT_CLIMBER_LEDS = LEDStrip.createCANdleLEDStrip(RIGHT_CLIMBER_INVERTED, RIGHT_CLIMBER_NUMBER_OF_LEDS, LED_OFFSET),
            LEFT_CLIMBER_LEDS = LEDStrip.createCANdleLEDStrip(LEFT_CLIMBER_INVERTED, LEFT_CLIMBER_NUMBER_OF_LEDS, RIGHT_CLIMBER_NUMBER_OF_LEDS + LED_OFFSET);

    public static final int DEFAULT_COMMAND_BREATHING_LEDS_AMOUNT = 7;
    public static final double DEFAULT_COMMAND_BREATHING_SPEED = 0.95;
    public static final boolean DEFAULT_COMMAND_BREATHING_IS_INVERTED = false;
    public static final double
            SCORING_BLINKING_SPEED = 0.5,
            INTAKE_GROUND_CORAL_BREATHING_SPEED = 0.3;
    public static final int SCORING_BREATHING_LEDS_AMOUNT = 5;

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
        CANdleLEDStrip.setTotalAmountOfLEDs(RIGHT_CLIMBER_NUMBER_OF_LEDS + LEFT_CLIMBER_NUMBER_OF_LEDS + LED_OFFSET);
    }
}