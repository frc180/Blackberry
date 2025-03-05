package frc.robot.subsystems.IntakeCoral;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeCoralIOTalonFXS implements IntakeCoralIO {

    TalonFXS intakeMotor;
    DigitalInput intakeSensor;
    VoltageOut voltageControl;

    StatusSignal<Voltage> voltageSignal;

    public IntakeCoralIOTalonFXS() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        intakeMotor = new TalonFXS(Constants.INTAKE_CORAL_TALON, Constants.CANIVORE);
        intakeMotor.getConfigurator().apply(config);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        
        intakeSensor = new DigitalInput(Constants.INTAKE_CORAL_SENSOR);   
        
        voltageControl = new VoltageOut(0);

        voltageSignal = trackSignal(intakeMotor.getMotorVoltage());

        // intakeMotor.optimizeBusUtilization(10, 0.1);
    }

    
    @Override
    public void update(IntakeIOInputs inputs) {
        inputs.coralSensor = false; //intakeSensor.get();
        inputs.voltage = voltageSignal.getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        intakeMotor.setControl(voltageControl.withOutput(speed * 12));
        SmartDashboard.putNumber("DEBUG Coral Requested Speed", speed);
    }
}
