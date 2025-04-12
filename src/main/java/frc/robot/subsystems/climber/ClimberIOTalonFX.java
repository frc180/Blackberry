package frc.robot.subsystems.climber;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {

    final TalonFX winchMotor;
    final DutyCycleEncoder absoluteEncoder;
    final VoltageOut voltageControl, grabberVoltage;

    TalonFX grabberMotor = null;
    StatusSignal<AngularVelocity> grabberVelocity = null;

    public ClimberIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        winchMotor = new TalonFX(Constants.CLIMBER_WINCH_TALON, Constants.CANIVORE);
        winchMotor.getConfigurator().apply(config);
        winchMotor.setNeutralMode(NeutralModeValue.Coast);

        config = new TalonFXConfiguration();
        grabberMotor = new TalonFX(Constants.CLIMBER_GRABBER_TALON, Constants.CANIVORE);
        grabberMotor.getConfigurator().apply(config);

        absoluteEncoder = new DutyCycleEncoder(Constants.DIO_INTAKE_ALGAE_ENCODER);

        voltageControl = new VoltageOut(0);
        grabberVoltage = new VoltageOut(0);

        if (grabberMotor != null) {
            grabberVelocity = trackSignal(grabberMotor.getVelocity());
        }
    }

    @Override
    public void update(ClimberInputs inputs) {
        inputs.jointPosition = absoluteEncoder.get();
        if (grabberVelocity != null) {
            inputs.grabberVelocity = grabberVelocity.getValueAsDouble();
        }
    }

    @Override
    public void setSpeed(double speed) {
        winchMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    @Override
    public void setGrabberSpeed(double speed) {
        if (grabberMotor != null) {
            grabberMotor.setControl(grabberVoltage.withOutput(speed * 12));
        }
    }
}
