package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
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

    public final LEDColor RED = new LEDColor(50, 0, 0, 255);
    // public final LEDColor BLUE = new LEDColor(0, 0, 255, 255); // primary blue
    public final LEDColor BLUE = new LEDColor(21, 46, 150, 255); // navy blue, b 99
    public final LEDColor GREEN = new LEDColor(0, 255, 0, 255);
    public final LEDColor YELLOW = new LEDColor(255, 255, 0, 255);
    public final LEDColor WHITE = new LEDColor(255, 255, 255, 255);
    public final LEDColor ALGAE = new LEDColor(0, 255, 30, 255);

    public final RainbowAnimation rainbow;
    public final SingleFadeAnimation blueFade, redFade, yellowFade, whiteFade, yellowFadeFast;
    public final TwinkleAnimation blueTwinkle, redTwinkle;
    public final ColorFlowAnimation blueFlow, redFlow, yellowFlow;
    public final LarsonAnimation yellowLarson;
    public final StrobeAnimation greenStrobe;

    private final int STRIP_LENGTH = 49;
    private final int STRIP_2_LENGTH = 47;
    private final int NUM_LEDS = 8 + STRIP_LENGTH + STRIP_2_LENGTH;
    private final int STRIP_OFFSET = 0;
    private final int NO_CANDLE_OFFSET = 8;
    private final CANdle candle;

    private Animation currentAnimation = null;
    private LEDColor currentColor = null;
    private LEDColor currentSplitColor = null;

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.disableWhenLOS = false;
        config.statusLedOffWhenActive = true;
        config.brightnessScalar = 0.2; // was 0.3
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
        yellowFadeFast = fade(YELLOW, 1);

        blueTwinkle = twinkle(BLUE, 0.25, TwinklePercent.Percent64);
        redTwinkle = twinkle(RED, 0.25, TwinklePercent.Percent64);

        blueFlow = colorFlow(BLUE, 0.5, Direction.Forward);
        redFlow = colorFlow(RED, 0.5, Direction.Forward);
        yellowFlow = colorFlow(YELLOW, 0.5, Direction.Forward);
        yellowLarson = larson(YELLOW, 0.5, 5, BounceMode.Front);

        greenStrobe = strobe(GREEN, 0.25);
    }

    public Command animate(Animation animation) {
        return run(() -> setAnimation(animation));
    }

    public Command color(LEDColor color) {
        return run(() -> setColor(color));
    }

    public void setAnimation(Animation animation) {
        setAnimation(animation, 0);
    }

    public void setAnimation(Animation animation, int slot) {
        if (animation != currentAnimation) {
            ErrorCode code = candle.animate(animation, slot);
            if (code == ErrorCode.OK) {
                currentAnimation = animation;
                currentColor = null;
                currentSplitColor = null;
            }
        }
    }

    public void setColor(LEDColor color) {
        if (color != currentColor) {
            candle.clearAnimation(0);
            ErrorCode code = candle.setLEDs(color.r, color.g, color.b, color.w, STRIP_OFFSET, NUM_LEDS);
            if (code == ErrorCode.OK) {
                currentColor = color;
                currentAnimation = null;
                currentSplitColor = null;
            }
        }
    }

    public void setSplitColor(LEDColor top, LEDColor bottom) {
        if (top != currentColor || bottom != currentSplitColor) {
            candle.clearAnimation(0);
            ErrorCode code1 = candle.setLEDs(bottom.r, bottom.g, bottom.b, bottom.w, 0, 24);
            ErrorCode code2 = candle.setLEDs(top.r, top.g, top.b, top.w, 24, 25);

            ErrorCode code3 =  candle.setLEDs(top.r, top.g, top.b, top.w, 49, 23);
            ErrorCode code4 =  candle.setLEDs(bottom.r, bottom.g, bottom.b, bottom.w, 72, 24);

            if (code1 == ErrorCode.OK && code2 == ErrorCode.OK && code3 == ErrorCode.OK && code4 == ErrorCode.OK) {
                currentColor = top;
                currentSplitColor = bottom;
                currentAnimation = null;
            }
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
        return new LarsonAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS - NO_CANDLE_OFFSET, mode, size, NO_CANDLE_OFFSET);
    }

    public StrobeAnimation strobe(LEDColor color, double speed) {
        return new StrobeAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, STRIP_OFFSET);
    }

    private record LEDColor(int r, int g, int b, int w) {}
}
