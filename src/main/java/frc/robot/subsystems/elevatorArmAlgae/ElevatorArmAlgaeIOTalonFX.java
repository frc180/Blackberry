package frc.robot.subsystems.elevatorArmAlgae;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorArmAlgaeIOTalonFX implements ElevatorArmAlgaeIO{

    TalonFX motor;
    DigitalInput sensor;

    VoltageOut voltageControl;
    
    public ElevatorArmAlgaeIOTalonFX() {
        motor = new TalonFX(Constants.ELEVATOR_ARM_ALGAE);
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

    private void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    private void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }
}
