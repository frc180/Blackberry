package com.spamrobotics.led;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleStrip extends SubsystemBase {

    private final CANdle candle;
    private final int startIndex;
    private final int endIndex;
    private final int animationSlot;

    private final SolidColor solidColor, firstHalfColor, secondHalfColor;
    private final SingleFadeAnimation fade;

    private boolean flipDirection = false;

    public CANdleStrip(CANdle candle, int startIndex, int endIndex, int animationSlot) {
        this.candle = candle;
        this.startIndex = startIndex;
        this.endIndex = endIndex;
        this.animationSlot = animationSlot;

        int halfLength = (endIndex - startIndex) / 2;
        solidColor = new SolidColor(startIndex, endIndex);
        firstHalfColor = new SolidColor(startIndex, startIndex + halfLength);
        secondHalfColor = new SolidColor(startIndex + halfLength + 1, endIndex);

        fade = new SingleFadeAnimation(startIndex, endIndex).withSlot(animationSlot);
    }

    public CANdleStrip withFlipDirection(boolean flipDirection) {
        this.flipDirection = flipDirection;
        return this;
    }

    public Command splitColor(RGBWColor first, RGBWColor second) {
        return run(() -> setSplitColor(first, second));
    }

    public Command solidColor(RGBWColor color) {
        return run(() -> setSolidColor(color));
    }

    public Command fade(RGBWColor color, double speed) {
        return run(() -> setFade(color, speed));
    }

    // public Command twinkle(TwinkleAnimation twinkleAnimation) {
    //     return run(() -> apply(twinkleAnimation));
    // }

    public void setSplitColor(RGBWColor first, RGBWColor second) {
        firstHalfColor.withColor(flipDirection ? second : first);
        secondHalfColor.withColor(flipDirection ? first : second);
        setControl(firstHalfColor);
        setControl(secondHalfColor);
    }

    public void setSolidColor(RGBWColor color) {
        setControl(solidColor.withColor(color));
    }

    public void setFade(RGBWColor color, double speed) {
        setControl(fade.withColor(color).withFrameRate(1000 * speed));
    }

    // public void apply(TwinkleAnimation twinkleAnimation) {
    //     setControl(twinkleAnimation.withLEDStartIndex(startIndex).withLEDEndIndex(endIndex).withSlot(animationSlot));
    // }

    private void setControl(ControlRequest control) {
        candle.setControl(control);
    }

    public static CANdleStrip ofLength(CANdle candle, int startIndex, int length, int animationSlot) {
        return new CANdleStrip(candle, startIndex, startIndex + length - 1, animationSlot);
    }
}
