package frc.robot.subsystems.coralIndexer;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class CoralIndexerIOTalonFXS implements CoralIndexerIO {
    
    TalonFXS motor;
    VoltageOut voltageControl;
    
    public CoralIndexerIOTalonFXS() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor = new TalonFXS(Constants.CORAL_INDEXER, Constants.CANIVORE);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);

        voltageControl = new VoltageOut(0);
    }

    @Override
    public void update(CoralIndexerInputs inputs) {
        // todo
    }

    @Override
    public void setSpeed(double speed) {
        motor.setControl(voltageControl.withOutput(speed * 12));
    }
}
