package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    private final int STRIP_LENGTH = 49;
    private final int STRIP_2_LENGTH = 47;
    private final int ALL_LEDS_INDEX = (8 + STRIP_LENGTH + STRIP_2_LENGTH) - 1;
    private final int NO_CANDLE_INDEX = 8;

    public final RGBWColor RED = new RGBWColor(50, 0, 0, 0);
    public final RGBWColor BLUE = new RGBWColor(21, 46, 150, 0);
    public final RGBWColor GREEN = new RGBWColor(0, 255, 0, 00);
    public final RGBWColor YELLOW = new RGBWColor(255, 255, 0, 0);
    public final RGBWColor WHITE = new RGBWColor(255, 255, 255, 0);
    public final RGBWColor ALGAE = new RGBWColor(0, 255, 30, 0);
    public final RGBWColor PURPLE = new RGBWColor(163, 49, 196, 0);

    private final EmptyAnimation[] emptyAnimations = new EmptyAnimation[8];
    private final SolidColor fullStripSolid = new SolidColor(0, ALL_LEDS_INDEX);

    private final SolidColor strip1TopSolid = new SolidColor(0, 23);
    private final SolidColor strip1BottomSolid = new SolidColor(24, STRIP_LENGTH - 1);

    private final SolidColor strip2TopSolid = new SolidColor(STRIP_LENGTH, STRIP_LENGTH + 23);
    private final SolidColor strip2BottomSolid = new SolidColor(STRIP_LENGTH + 24, STRIP_2_LENGTH - 1);

    // ErrorCode code1 = candle.setLEDs(bottom.r, bottom.g, bottom.b, bottom.w, 0, 24);
    // ErrorCode code2 = candle.setLEDs(top.r, top.g, top.b, top.w, 24, 25);

    // ErrorCode code3 =  candle.setLEDs(top.r, top.g, top.b, top.w, 49, 23);
    // ErrorCode code4 =  candle.setLEDs(bottom.r, bottom.g, bottom.b, bottom.w, 72, 24);


    public final RainbowAnimation rainbow;
    public final SingleFadeAnimation blueFade, redFade, yellowFade, whiteFade, yellowFadeFast, purpleFade;
    public final TwinkleAnimation blueTwinkle, redTwinkle;
    public final ColorFlowAnimation blueFlow, redFlow, yellowFlow;
    public final LarsonAnimation yellowLarson;
    public final StrobeAnimation greenStrobe;

    private final CANdle candle;

    private ControlRequest currentAnimation = null;
    private RGBWColor currentColor = null;
    private RGBWColor currentSplitColor = null;

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.2;
        config.LED.StripType = StripTypeValue.GRB;
        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
        candle = new CANdle(Constants.CANDLE, Constants.CANIVORE);
        candle.getConfigurator().apply(config);

        // Clear all previous animations
        for (int i = 0; i < 8; ++i) {
            emptyAnimations[i] = new EmptyAnimation(i);
            candle.setControl(emptyAnimations[i]);
        }

        rainbow = new RainbowAnimation(0, ALL_LEDS_INDEX);

        blueFade = fade(BLUE, 0.5);
        redFade = fade(RED, 0.5);
        yellowFade = fade(YELLOW, 0.5);
        purpleFade = fade(PURPLE, 1);
        whiteFade = fade(WHITE, 1);
        yellowFadeFast = fade(YELLOW, 1);

        blueTwinkle = twinkle(BLUE, 0.25, 0.64);
        redTwinkle = twinkle(RED, 0.25, 0.64);

        blueFlow = colorFlow(BLUE, 0.7, AnimationDirectionValue.Forward);
        redFlow = colorFlow(RED, 0.7, AnimationDirectionValue.Forward);
        yellowFlow = colorFlow(YELLOW, 0.5, AnimationDirectionValue.Forward);
        
        yellowLarson = new LarsonAnimation(NO_CANDLE_INDEX, ALL_LEDS_INDEX)
                            .withColor(YELLOW)
                            .withFrameRate(500)
                            .withSize(5)
                            .withBounceMode(LarsonBounceValue.Front);

        greenStrobe = strobe(GREEN, 0.25);
    }

    public Command animate(ControlRequest animation) {
        return run(() -> setAnimation(animation));
    }

    public Command color(RGBWColor color) {
        return run(() -> setColor(color));
    }

    public void setAnimation(ControlRequest animation) {
        if (animation != currentAnimation) {
            StatusCode code = candle.setControl(animation);
            if (code == StatusCode.OK) {
                clearState();
                currentAnimation = animation;
            }
        }
    }

    // public void setDualAnimation(Animation animation1, Animation animation2) {
    //     if (currentAnimation != animation1 || currentAnimation2 != animation2) {
    //         clearAnimations();
    //         ErrorCode code1 = candle.animate(animation1, 0);
    //         ErrorCode code2 = candle.animate(animation2, 1);

    //         if (code1 == ErrorCode.OK && code2 == ErrorCode.OK) {
    //             clearState();
    //             currentAnimation = animation1;
    //             currentAnimation2 = animation2;
    //         }
    //     }
    // }

    public void setColor(RGBWColor color) {
        if (color != currentColor) {
            clearAnimations();
            StatusCode code = candle.setControl(fullStripSolid.withColor(color));
            if (code == StatusCode.OK) {
                clearState();
                currentColor = color;
            }
        }
    }

    public void setSplitColor(RGBWColor top, RGBWColor bottom) {
        if (top != currentColor || bottom != currentSplitColor) {
            clearAnimations();

            StatusCode code1 = candle.setControl(strip1TopSolid.withColor(top));
            StatusCode code2 = candle.setControl(strip1BottomSolid.withColor(bottom));
            
            StatusCode code3 = candle.setControl(strip2TopSolid.withColor(top));
            StatusCode code4 = candle.setControl(strip2BottomSolid.withColor(bottom));

            if (code1 == StatusCode.OK && code2 == StatusCode.OK && code3 == StatusCode.OK && code4 == StatusCode.OK) {
                clearState();
                currentColor = top;
                currentSplitColor = bottom;
            }
        }
    }

    private void clearAnimations() {
        candle.setControl(emptyAnimations[0]);
    }

    private void clearState() {
        currentAnimation = null;
        currentColor = null;
        currentSplitColor = null;
    }

    public SingleFadeAnimation fade(RGBWColor color, double speedPercent) {
        return new SingleFadeAnimation(0, ALL_LEDS_INDEX)
                .withColor(color)
                .withFrameRate(1000 * speedPercent);
    }

    public StrobeAnimation strobe(RGBWColor color, double speedPercent) {
        return new StrobeAnimation(0, ALL_LEDS_INDEX)
                .withColor(color)
                .withFrameRate(1000 * speedPercent);
    }

    public TwinkleAnimation twinkle(RGBWColor color, double speedPercent, double twinklePercent) {
        return new TwinkleAnimation(0, ALL_LEDS_INDEX)
                .withColor(color)
                .withFrameRate(1000 * speedPercent)
                .withMaxLEDsOnProportion(twinklePercent);
    }

    public TwinkleOffAnimation twinkleOff(RGBWColor color, double speedPercent, double twinklePercent) {
        return new TwinkleOffAnimation(0, ALL_LEDS_INDEX)
                .withColor(color)
                .withFrameRate(1000 * speedPercent)
                .withMaxLEDsOnProportion(twinklePercent);
    }

    public ColorFlowAnimation colorFlow(RGBWColor color, double speedPercent, AnimationDirectionValue direction) {
        return new ColorFlowAnimation(0, ALL_LEDS_INDEX)
                .withColor(color)
                .withFrameRate(1000 * speedPercent)
                .withDirection(direction);
    }
}
