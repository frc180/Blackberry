package frc.robot.subsystems.IntakeAlgae;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


public class IntakeAlgaeIOTalonFXS implements IntakeAlgaeIO {
    
    TalonFX intakeMotor;

    public IntakeAlgaeIOTalonFXS() {
        intakeMotor = new TalonFX(Constants.INTAKE_ALGAE_TALON);

    }
    
    @Override
    public void startRollers() {
        intakeMotor.set(1);
    }

    @Override
    public void stopRollers() {
        intakeMotor.stopMotor();
    }

}
