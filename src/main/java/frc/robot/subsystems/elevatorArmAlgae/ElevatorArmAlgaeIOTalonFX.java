package frc.robot.subsystems.elevatorArmAlgae;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorArmAlgaeIOTalonFX implements ElevatorArmAlgaeIO{

    TalonFXS motor;
    DigitalInput sensor;

    VoltageOut voltageControl;
    
    public ElevatorArmAlgaeIOTalonFX() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        motor = new TalonFXS(Constants.ELEVATOR_ARM_ALGAE, Constants.CANIVORE);
        motor.getConfigurator().apply(config);
        sensor = new DigitalInput(Constants.ELEVATOR_ARM_ALGAE_SENSOR);
        voltageControl = new VoltageOut(0);
    }

    @Override
    public void update(ElevatorArmAlgaeInputs inputs) {
        inputs.hasAlgae = sensor.get();
    }

    @Override
    public void run() {
        setSpeed(1);
    }

    @Override
    public void stop() {
        setSpeed(0);
    }

    @Override
    public void reverse() {
        setSpeed(-1);
    }

    @Override
    public void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    private void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
   }

    public void runMotorTest() {
        System.out.println("elevator arm algae running");
        motor.set(0.25);
    }
}
