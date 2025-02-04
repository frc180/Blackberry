package frc.robot.subsystems.IntakeAlgae;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;


public class IntakeAlgaeIOTalonFXS implements IntakeAlgaeIO {
    
    TalonFX intakeMotor;
    DigitalInput sensor;

    public IntakeAlgaeIOTalonFXS() {
        intakeMotor = new TalonFX(Constants.INTAKE_ALGAE_TALON);
        sensor = new DigitalInput(Constants.INTALE_ALGAE_SENSOR);

    }
    
    @Override
    public void startRollers() {
        intakeMotor.set(1);
    }

    @Override
    public void stopRollers() {
        intakeMotor.stopMotor();
    }

    @Override
    public void update(IntakeAlgaeIOInputs inputs) {
        inputs.hasAlgae = sensor.get();
    }

}
