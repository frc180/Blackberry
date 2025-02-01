package frc.robot.subsystems.IntakeCoralPivot;

import java.util.List;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Robot;


public class IntakeCoralPivotIOTalonFXS implements IntakeCoralPivotIO {

    TalonFX motor;
    MotionMagicVoltage motionMagicControl;
    VoltageOut voltageControl;
    
    public IntakeCoralPivotIOTalonFXS() {
        motor = new TalonFX(Constants.INTAKE_CORAL_PIVOT_TALON);
        motor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kG = 0;
        config.Slot0.kV = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotionMagic.MotionMagicAcceleration = 0;
        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicJerk = 0;

        motor.getConfigurator().apply(config);


    }

    @Override
    public void setIntakePosition(double position){
        motor.setControl(motionMagicControl.withPosition(position));
    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void update(IntakeCoralPivotIOInputs inputs) {
        inputs.position = motor.getPosition().getValueAsDouble();
        inputs.target = motor.getClosedLoopReference().getValueAsDouble();
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    }

    public void stop() {
        motor.stopMotor();
    }

}
