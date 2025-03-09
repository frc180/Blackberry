package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    private final int NUM_LEDS = 8;
    private final int STRIP_OFFSET = 0;

    private CANdle candle = new CANdle(Constants.CANDLE);

    public final RainbowAnimation rainbow;
    public final SingleFadeAnimation blueFade, redFade, yellowFade;
    public final TwinkleAnimation blueTwinkle;

    public LEDSubsystem() {
        rainbow = new RainbowAnimation(1, 1, NUM_LEDS, false, STRIP_OFFSET);
        blueFade = new SingleFadeAnimation(0, 0, 255, 255, 1, NUM_LEDS, STRIP_OFFSET);
        redFade = new SingleFadeAnimation(255, 0, 0, 255, 1, NUM_LEDS, STRIP_OFFSET);
        yellowFade = new SingleFadeAnimation(255, 0, 0, 255, 1, NUM_LEDS, STRIP_OFFSET);
        blueTwinkle = new TwinkleAnimation(0, 0, 255, 255, 1, NUM_LEDS, TwinklePercent.Percent30, STRIP_OFFSET);
    }

    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }
}
