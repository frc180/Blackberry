package frc.robot.subsystems.IntakeCoralPivot;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeCoralPivotIOTalonFXS implements IntakeCoralPivotIO {
    final double PIVOT_GEARING = 1; // TODO - get real number

    TalonFXS motor;
    MotionMagicExpoVoltage motionMagicControl;
    VoltageOut voltageControl;

    // Status signals
    StatusSignal<Angle> positionSignal;
    StatusSignal<Voltage> voltageSignal;
    StatusSignal<Double> targetSignal;
    StatusSignal<Current> supplyCurrentSignal;
    StatusSignal<Current> torqueCurrentSignal;

    // Simulation-only variables
    TalonFXSSimState motorSim = null;
    double simulatedPosition = Units.degreesToRotations(-90);
    
    public IntakeCoralPivotIOTalonFXS() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
        if (Robot.isReal()) {
            // config.ExternalFeedback.RotorToSensorRatio = 1; // use if the encoder is not already in rotations
            config.ExternalFeedback.SensorToMechanismRatio = PIVOT_GEARING;
            config.Slot0.kP = 0;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 0;
            config.MotionMagic.MotionMagicExpo_kV = 2;
            config.MotionMagic.MotionMagicExpo_kA = 0;
        } else {
            config.Slot0.kP = 1440;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 0;
            config.MotionMagic.MotionMagicExpo_kV = .05;
            config.MotionMagic.MotionMagicExpo_kA = 0;
        }
        config.MotionMagic.MotionMagicJerk = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor = new TalonFXS(Constants.INTAKE_CORAL_PIVOT_TALON, Constants.CANIVORE);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);

        voltageControl = new VoltageOut(0);
        motionMagicControl = new MotionMagicExpoVoltage(0);

        positionSignal = trackSignal(motor.getPosition());
        voltageSignal = trackSignal(motor.getMotorVoltage());
        targetSignal = trackSignal(motor.getClosedLoopReference());
        supplyCurrentSignal = trackSignal(motor.getSupplyCurrent());
        torqueCurrentSignal = trackSignal(motor.getTorqueCurrent());

        if (Robot.isReal()) return;

        motorSim = motor.getSimState();
    }

    @Override
    public void update(IntakeCoralPivotIOInputs inputs) {
        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
        inputs.torqueCurrent = torqueCurrentSignal.getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        simulatedPosition += motorSim.getMotorVoltage() * Units.degreesToRotations(1.5);
        motorSim.setRawRotorPosition(simulatedPosition);
    }

    @Override
    public void setIntakePosition(double position){
        motor.setControl(motionMagicControl.withPosition(position));
    }

    @Override
    public void setSpeed(double speed) {
        motor.setControl(voltageControl.withOutput(speed * 12));
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void runMotorTest() {
        System.out.println("coral intake pivot running");
        motor.set(0.25);
    }
}
