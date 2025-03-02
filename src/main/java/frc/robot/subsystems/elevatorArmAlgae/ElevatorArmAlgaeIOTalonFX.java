package frc.robot.subsystems.elevatorArmAlgae;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ElevatorArmAlgaeIOTalonFX implements ElevatorArmAlgaeIO{

    TalonFXS motor;
    CANrange canrange;
    VoltageOut voltageControl;

    StatusSignal<Distance> distanceSignal;
    
    public ElevatorArmAlgaeIOTalonFX() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        motor = new TalonFXS(Constants.ELEVATOR_ARM_ALGAE, Constants.CANIVORE);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
        voltageControl = new VoltageOut(0);

        // CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
        // rangeConfig.FovParams.FOVRangeX = 6.75;
        // rangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        // canrange = new CANrange(Constants.ALGAE_ARM_CANRANGE, Constants.CANIVORE);
        // canrange.getConfigurator().apply(rangeConfig);
        // distanceSignal = trackSignal(canrange.getDistance());
    }

    @Override
    public void update(ElevatorArmAlgaeInputs inputs) {
        inputs.hasAlgae = false;
        // double distance = distanceSignal.getValueAsDouble();
        // inputs.distance = distance;
        // inputs.hasAlgae = distance < 0.4;
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
