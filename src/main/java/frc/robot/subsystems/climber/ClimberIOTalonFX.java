package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {

    final TalonFX winchMotor;
    final DutyCycleEncoder absoluteEncoder;
    final VoltageOut voltageControl;

    public ClimberIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        winchMotor = new TalonFX(Constants.CLIMBER_WINCH_TALON, Constants.CANIVORE);
        winchMotor.getConfigurator().apply(config);
        winchMotor.setNeutralMode(NeutralModeValue.Brake);

        absoluteEncoder = new DutyCycleEncoder(Constants.DIO_INTAKE_ALGAE_ENCODER);

        voltageControl = new VoltageOut(0);
    }

    @Override
    public void update(ClimberInputs inputs) {
        inputs.jointPosition = absoluteEncoder.get();
    }

    @Override
    public void setSpeed(double speed) {
        winchMotor.setControl(voltageControl.withOutput(speed * 12));
    }
}
