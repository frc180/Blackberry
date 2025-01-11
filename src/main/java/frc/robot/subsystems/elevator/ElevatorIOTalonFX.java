package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

    TalonFX motorA, motorB;
    PositionVoltage positionControl;


    public ElevatorIOTalonFX() {
        motorA = new TalonFX(Constants.ELEVATOR_TALON_A);
        motorB = new TalonFX(Constants.ELEVATOR_TALON_B);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 1; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        motorA.getConfigurator().apply(slot0Configs);
        motorB.getConfigurator().apply(slot0Configs);

        positionControl = new PositionVoltage(0);
    }

    @Override
    public void setPower(double power) {
        motorA.set(power);
        motorB.set(power);
    }

    @Override
    public void setPosition(double encoderPosition) {
        positionControl.withPosition(encoderPosition);
        motorA.setControl(positionControl);
        motorB.setControl(positionControl);
    }

    @Override
    public void update() {
    }
}
