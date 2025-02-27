package frc.robot.subsystems.elevatorArmAlgae;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ElevatorArmAlgaeIOTalonFX implements ElevatorArmAlgaeIO{

    TalonFXS motor;
    // DigitalInput sensor;

    VoltageOut voltageControl;
    
    public ElevatorArmAlgaeIOTalonFX() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        motor = new TalonFXS(Constants.ELEVATOR_ARM_ALGAE, Constants.CANIVORE);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
        // sensor = new DigitalInput(Constants.ELEVATOR_ARM_ALGAE_SENSOR);
        voltageControl = new VoltageOut(0);
    }

    @Override
    public void update(ElevatorArmAlgaeInputs inputs) {
        // inputs.hasAlgae = sensor.get();
        inputs.hasAlgae = false;
    }

    @Override
    public void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

    private void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
   }
}
