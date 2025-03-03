package frc.robot.subsystems.IntakeCoral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class IntakeCoralIOSpark implements IntakeCoralIO {

    SparkMax motor;
    DigitalInput intakeSensor;

    public IntakeCoralIOSpark() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake)
                   .voltageCompensation(12);

        motor = new SparkMax(Constants.INTAKE_CORAL_SPARK, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        intakeSensor = new DigitalInput(Constants.INTAKE_CORAL_SENSOR);
    }

    @Override
    public void update(IntakeIOInputs inputs) {
        inputs.coralSensor = false; //intakeSensor.get();
        inputs.voltage = motor.getAppliedOutput() * 12;
    }

    @Override
    public void setSpeed(double speed) {
        motor.setVoltage(speed * 12);
    }
}
