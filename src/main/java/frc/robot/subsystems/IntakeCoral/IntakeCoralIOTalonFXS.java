package frc.robot.subsystems.IntakeCoral;

import java.util.List;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeCoralIOTalonFXS implements IntakeCoralIO{

    TalonFX intakeMotor;
    DigitalInput intakeSensor;
    


    public IntakeCoralIOTalonFXS() {
        intakeMotor = new TalonFX(Constants.INTAKE_CORAL_TALON);
        intakeMotor.setInverted(false);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        
        intakeSensor = new DigitalInput(Constants.INTAKE_CORAL_SENSOR);       
    }

    
    @Override
    public void update(IntakeIOInputs inputs) {
        inputs.coralSensor = intakeSensor.get();
        inputs.voltage = intakeMotor.getMotorVoltage(true).getValueAsDouble();
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
