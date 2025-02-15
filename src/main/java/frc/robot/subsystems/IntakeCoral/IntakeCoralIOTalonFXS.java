package frc.robot.subsystems.IntakeCoral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeCoralIOTalonFXS implements IntakeCoralIO{

    TalonFX intakeMotor;
    DigitalInput intakeSensor;
    VoltageOut voltageControl;

    public IntakeCoralIOTalonFXS() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        intakeMotor = new TalonFX(Constants.INTAKE_CORAL_TALON, Constants.CANIVORE);
        intakeMotor.getConfigurator().apply(config);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        
        intakeSensor = new DigitalInput(Constants.INTAKE_CORAL_SENSOR);   
        
        voltageControl = new VoltageOut(0);
    }

    
    @Override
    public void update(IntakeIOInputs inputs) {
        inputs.coralSensor = intakeSensor.get();
        inputs.voltage = intakeMotor.getMotorVoltage(true).getValueAsDouble();
    }
    
    @Override
    public void startRollers() {
        setSpeed(1);
    }

    @Override
    public void stopRollers() {
        setSpeed(0);
    }

    private void setSpeed(double speed) {
        intakeMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    @Override
    public void runMotorTest() {
        System.out.println("coral intake running");
        intakeMotor.set(0.25);
    }
}
