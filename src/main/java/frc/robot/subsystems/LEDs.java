package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.spamrobotics.led.CANdleStrip;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class LEDs {

    private static final int ONBOARD_LENGTH = 8;
    private static final int LEFT_STRIP_LENGTH = 49;
    private static final int RIGHT_STRIP_LENGTH = 47;

    private static final int FINAL_INDEX = ONBOARD_LENGTH + LEFT_STRIP_LENGTH + RIGHT_STRIP_LENGTH - 1;

    private final CANdle candle;
    private final SolidColor solidColor;

    public final CANdleStrip onboardStrip, leftStrip, rightStrip;
    public final List<CANdleStrip> strips;
    
    public LEDs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.2;
        config.LED.StripType = StripTypeValue.GRB;
        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
        candle = new CANdle(Constants.CANDLE, Constants.CANIVORE);
        candle.getConfigurator().apply(config);

        solidColor = new SolidColor(0, FINAL_INDEX);

        onboardStrip = new CANdleStrip(
            candle,
            0, 
            ONBOARD_LENGTH - 1, 
            0
        );
        leftStrip = new CANdleStrip(
            candle,
            ONBOARD_LENGTH,
            ONBOARD_LENGTH + LEFT_STRIP_LENGTH - 1, 
            1
        );
        rightStrip = new CANdleStrip(
            candle,
            ONBOARD_LENGTH + LEFT_STRIP_LENGTH, 
            FINAL_INDEX, 
            2
        ).withFlipDirection(true);
        strips = List.of(onboardStrip, leftStrip, rightStrip);
    }

    // public Command solidColor(RGBWColor color) {
    //     solidColor.withColor(color);
    //     return Commands.parallel(
    //         leftStrip.solidColor(solidColor),
    //         rightStrip.solidColor(solidColor)
    //     );
    // }

    public Command color(RGBWColor color) {
        return withStrips(Commands.run(() -> setColor(color)));
    }

    public Command leftRightColor(RGBWColor left, RGBWColor right) {
        return Commands.parallel(
            leftStrip.solidColor(left),
            rightStrip.solidColor(right)
        );
    }

    public Command leftRideFade(RGBWColor left, RGBWColor right, double speed) {
        return Commands.parallel(
            leftStrip.fade(left, speed),
            rightStrip.fade(right, speed)
        );
    }

    public Command splitColor(RGBWColor first, RGBWColor second) {
        return Commands.parallel(
            leftStrip.splitColor(first, second),
            rightStrip.splitColor(first, second)
        );
    }

    public void setColor(RGBWColor color) {
        candle.setControl(solidColor.withColor(color));
    }

    public void setLeftRightFade(RGBWColor left, RGBWColor right, double speed) {
        leftStrip.setFade(left, speed);
        rightStrip.setFade(right, speed);
    }

    public void setSplitColor(RGBWColor first, RGBWColor second) {
        leftStrip.setSplitColor(first, second);
        rightStrip.setSplitColor(first, second);
    }

    public void setDefaultCommands(Supplier<Command> commandGenerator) {
        strips.forEach(strip -> {
            strip.setDefaultCommand(withRequirements(commandGenerator.get(), strip));
        });
    }

    private Command withRequirements(Command command, Subsystem... requirements) {
        command.addRequirements(requirements);
        return command;
    }

    private Command withStrips(Command command) {
        return withRequirements(command, onboardStrip, leftStrip, rightStrip);
    }
}
