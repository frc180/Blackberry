package frc.robot.subsystems.IntakeAlgae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeAlgaeIOTalonFXS implements IntakeAlgaeIO {
    
    TalonFX intakeMotor;
    VoltageOut voltageControl; 

    public IntakeAlgaeIOTalonFXS() {
        intakeMotor = new TalonFX(Constants.INTAKE_ALGAE_TALON);
        TalonFXConfiguration config = new TalonFXConfiguration();
        intakeMotor.getConfigurator().apply(config);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        voltageControl = new VoltageOut(0);
    }
    
    @Override
    public void startRollers() {
        setSpeed(1);
    }

    @Override
    public void stopRollers() {
        setSpeed(0);
    }

    /**
     * Sets the speed (-1 to 1) of the intake
     */
    private void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    private void setVoltage(double voltage) {
        intakeMotor.setControl(voltageControl.withOutput(voltage));
    }
}
