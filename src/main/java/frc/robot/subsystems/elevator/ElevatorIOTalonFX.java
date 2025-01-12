package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

    TalonFX motorA, motorB;
    PositionVoltage positionControl;
    VoltageOut voltageControl;
    Follower followerControl;

    public ElevatorIOTalonFX() {
        motorA = new TalonFX(Constants.ELEVATOR_TALON_A);
        motorB = new TalonFX(Constants.ELEVATOR_TALON_B);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 5; // An error of 1 rps results in 0.55 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
        slot0Configs.kG = 0; // no gravity compensation
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        motorA.getConfigurator().apply(slot0Configs);
        motorB.getConfigurator().apply(slot0Configs);

        positionControl = new PositionVoltage(0);
        voltageControl = new VoltageOut(0);
        followerControl = new Follower(Constants.ELEVATOR_TALON_A, true);

        motorB.setControl(followerControl);
    }

    @Override
    public void setPower(double power) {
        motorA.setControl(voltageControl.withOutput(power * 12));
    }

    @Override
    public void setPosition(double encoderPosition) {
        motorA.setControl(positionControl.withPosition(encoderPosition));
    }

    @Override
    public void update(ElevatorIOInputs inputs) {
        inputs.position = motorA.getPosition(true).getValueAsDouble();
        inputs.speed = motorA.getMotorVoltage(true).getValueAsDouble();
    }
}
