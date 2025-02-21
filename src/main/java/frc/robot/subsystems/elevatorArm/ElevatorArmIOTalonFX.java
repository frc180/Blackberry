package frc.robot.subsystems.elevatorArm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import frc.robot.Constants;

public class ElevatorArmIOTalonFX implements ElevatorArmIO {

    TalonFXS rollerMotor;
    CANdi candiA, candiB;
    VoltageOut voltageControl;

    StatusSignal<Boolean> frontSensorSignal, middleSensorSignal, backSensorSignal;

    public ElevatorArmIOTalonFX() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        rollerMotor = new TalonFXS(Constants.ELEVATOR_ARM_TALON, Constants.CANIVORE);
        rollerMotor.getConfigurator().apply(config);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);

        voltageControl = new VoltageOut(0);

        CANdiConfiguration candiConfig = new CANdiConfiguration();
        candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotHigh;
        candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        candiConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenNotHigh;
        candiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
        candiA = new CANdi(Constants.ELEVATOR_ARM_CANDI_A, Constants.CANIVORE);
        candiB = new CANdi(Constants.ELEVATOR_ARM_CANDI_B, Constants.CANIVORE);
        candiA.getConfigurator().apply(candiConfig);
        candiB.getConfigurator().apply(candiConfig);

        frontSensorSignal = candiA.getS2Closed();
        middleSensorSignal = candiA.getS1Closed();
        backSensorSignal = candiB.getS1Closed();
    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(frontSensorSignal, middleSensorSignal, backSensorSignal);
        inputs.frontCoralSensor = frontSensorSignal.getValueAsDouble() == 1;
        inputs.middleCoralSensor = middleSensorSignal.getValueAsDouble() == 1;
        inputs.backCoralSensor = backSensorSignal.getValueAsDouble() == 1;
        inputs.voltage = rollerMotor.getMotorVoltage(true).getValueAsDouble();
    }

    @Override
    public void run() {
        setSpeed(1);
    }

    @Override
    public void reverse() {
        setSpeed(-1);
    }

    @Override
    public void stop() {
        setSpeed(0);
    }

    @Override
    public void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    private void setVoltage(double volts) {
        rollerMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void runMotorTest() {
        System.out.println("elevator arm running");
        rollerMotor.set(0.25);
    }
}
