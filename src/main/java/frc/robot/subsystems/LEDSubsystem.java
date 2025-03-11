package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    private final int NUM_LEDS = 8 + 60;
    private final int STRIP_OFFSET = 0;
    private final CANdle candle;

    public final LEDColor RED = new LEDColor(255, 0, 0, 255);
    // public final LEDColor BLUE = new LEDColor(0, 0, 255, 255); // primary blue
    public final LEDColor BLUE = new LEDColor(21, 46, 99, 255); // navy blue
    public final LEDColor GREEN = new LEDColor(0, 255, 0, 255);
    public final LEDColor YELLOW = new LEDColor(255, 255, 0, 255);

    public final RainbowAnimation rainbow;
    public final SingleFadeAnimation blueFade, redFade, yellowFade;
    public final TwinkleAnimation blueTwinkle, redTwinkle;
    public final ColorFlowAnimation yellowFlow;
    public final LarsonAnimation yellowLarson;

    public LEDSubsystem() {
        candle = new CANdle(Constants.CANDLE);

        rainbow = new RainbowAnimation(1, 1, NUM_LEDS, false, STRIP_OFFSET);

        blueFade = fade(BLUE, 1);
        redFade = fade(RED, 1);
        yellowFade = fade(YELLOW, 1);

        blueTwinkle = twinkle(BLUE, 1, TwinklePercent.Percent30);
        redTwinkle = twinkle(RED, 1, TwinklePercent.Percent30);

        yellowFlow = colorFlow(YELLOW, 0.5, Direction.Forward);
        yellowLarson = larson(YELLOW, 1, 3, BounceMode.Front);
    }

    public Command animate(Animation animation) {
        return run(() -> setAnimation(animation));
    }

    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }

    public SingleFadeAnimation fade(LEDColor color, double speed) {
        return new SingleFadeAnimation(color.r, color.g, color.b, color.w, 1, NUM_LEDS, STRIP_OFFSET);
    }

    public TwinkleAnimation twinkle(LEDColor color, double speed, TwinklePercent percent) {
        return new TwinkleAnimation(color.r, color.g, color.b, color.w, 1, NUM_LEDS, percent, STRIP_OFFSET);
    }

    public ColorFlowAnimation colorFlow(LEDColor color, double speed, Direction direction) {
        return new ColorFlowAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, direction, STRIP_OFFSET);
    }

    public LarsonAnimation larson(LEDColor color, double speed, int size, BounceMode mode) {
        return new LarsonAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, mode, size, STRIP_OFFSET);
    }

    private record LEDColor(int r, int g, int b, int w) {}
}
