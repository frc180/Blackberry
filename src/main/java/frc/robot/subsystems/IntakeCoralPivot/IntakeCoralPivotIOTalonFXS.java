package frc.robot.subsystems.IntakeCoralPivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeCoralPivotIOTalonFXS implements IntakeCoralPivotIO {

    TalonFX motor;
    MotionMagicExpoVoltage motionMagicControl;
    VoltageOut voltageControl;

    // Simulation-only variables
    TalonFXSimState motorSim = null;
    double simulatedPosition = Units.degreesToRotations(-90);
    
    public IntakeCoralPivotIOTalonFXS() {
        motor = new TalonFX(Constants.INTAKE_CORAL_PIVOT_TALON, Constants.CANIVORE);
        motor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1440;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kG = 0;
        config.Slot0.kV = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotionMagic.MotionMagicAcceleration = 999;
        // config.MotionMagic.MotionMagicCruiseVelocity = 999;
        config.MotionMagic.MotionMagicExpo_kV = .05;
        config.MotionMagic.MotionMagicExpo_kA = 0;
        config.MotionMagic.MotionMagicJerk = 0;
        motor.getConfigurator().apply(config);

        motionMagicControl = new MotionMagicExpoVoltage(0);

        if (Robot.isReal()) return;

        motorSim = motor.getSimState();
    }

    @Override
    public void setIntakePosition(double position){
        motor.setControl(motionMagicControl.withPosition(position));
    }

    @Override
    public void simulationPeriodic() {
        simulatedPosition += motorSim.getMotorVoltage() * Units.degreesToRotations(1.5);
        motorSim.setRawRotorPosition(simulatedPosition);
    }

    @Override
    public void update(IntakeCoralPivotIOInputs inputs) {
        inputs.position = motor.getPosition().getValueAsDouble();
        inputs.target = motor.getClosedLoopReference().getValueAsDouble();
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
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
