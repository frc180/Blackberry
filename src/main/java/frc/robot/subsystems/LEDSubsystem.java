package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    public final LEDColor RED = new LEDColor(255, 0, 0, 255);
    // public final LEDColor BLUE = new LEDColor(0, 0, 255, 255); // primary blue
    public final LEDColor BLUE = new LEDColor(21, 46, 99, 255); // navy blue
    public final LEDColor GREEN = new LEDColor(0, 255, 0, 255);
    public final LEDColor YELLOW = new LEDColor(255, 255, 0, 255);
    public final LEDColor WHITE = new LEDColor(255, 255, 255, 255);
    public final LEDColor ALGAE = new LEDColor(53, 202, 183, 255);

    public final RainbowAnimation rainbow;
    public final SingleFadeAnimation blueFade, redFade, yellowFade, whiteFade;
    public final TwinkleAnimation blueTwinkle, redTwinkle;
    public final ColorFlowAnimation yellowFlow;
    public final LarsonAnimation yellowLarson;

    private final int NUM_LEDS = 8 + 60;
    private final int STRIP_OFFSET = 0;
    private final int NO_CANDLE_OFFSET = 8;
    private final CANdle candle;

    private Animation currentAnimation = null;
    private LEDColor currentColor = null;
    private LEDColor currentSplitColor = null;

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.disableWhenLOS = true;
        config.statusLedOffWhenActive = true;
        config.brightnessScalar = 0.4;
        config.v5Enabled = true;
        config.stripType = LEDStripType.GRB;
        config.vBatOutputMode = VBatOutputMode.Off;
        candle = new CANdle(Constants.CANDLE, Constants.CANIVORE);
        candle.configAllSettings(config);

        rainbow = new RainbowAnimation(1, 1, NUM_LEDS, false, STRIP_OFFSET);

        blueFade = fade(BLUE, 0.25);
        redFade = fade(RED, 0.25);
        yellowFade = fade(YELLOW, 0.25);
        whiteFade = fade(WHITE, 1);

        blueTwinkle = twinkle(BLUE, 0.25, TwinklePercent.Percent88);
        redTwinkle = twinkle(RED, 0.25, TwinklePercent.Percent88);

        yellowFlow = colorFlow(YELLOW, 0.5, Direction.Forward);
        yellowLarson = larson(YELLOW, 0.5, 3, BounceMode.Front);
    }

    public Command animate(Animation animation) {
        return run(() -> setAnimation(animation));
    }

    public Command color(LEDColor color) {
        return run(() -> setColor(color));
    }

    public void setAnimation(Animation animation) {
        if (animation != currentAnimation) {
            candle.animate(animation);
            currentAnimation = animation;
            currentColor = null;
            currentSplitColor = null;
        }
    }

    public void setColor(LEDColor color) {
        if (color != currentColor) {
            candle.setLEDs(color.r, color.g, color.b, color.w, STRIP_OFFSET, NUM_LEDS);
            currentColor = color;
            currentAnimation = null;
            currentSplitColor = null;
        }
    }

    public void setSplitColor(LEDColor top, LEDColor bottom) {
        if (top != currentColor || bottom != currentSplitColor) {
            candle.setLEDs(top.r, top.g, top.b, top.w, 0, 38);
            candle.setLEDs(bottom.r, bottom.g, bottom.b, bottom.w, 38, 30);
            currentColor = top;
            currentSplitColor = bottom;
            currentAnimation = null;
        }
    }

    public SingleFadeAnimation fade(LEDColor color, double speed) {
        return new SingleFadeAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, STRIP_OFFSET);
    }

    public TwinkleAnimation twinkle(LEDColor color, double speed, TwinklePercent percent) {
        return new TwinkleAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, percent, STRIP_OFFSET);
    }

    public TwinkleOffAnimation twinkleOff(LEDColor color, double speed, TwinkleOffPercent percent) {
        return new TwinkleOffAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, percent, STRIP_OFFSET);
    }

    public ColorFlowAnimation colorFlow(LEDColor color, double speed, Direction direction) {
        return new ColorFlowAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, direction, STRIP_OFFSET);
    }

    public LarsonAnimation larson(LEDColor color, double speed, int size, BounceMode mode) {
        return new LarsonAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, mode, size, NO_CANDLE_OFFSET);
    }

    private record LEDColor(int r, int g, int b, int w) {}
}
