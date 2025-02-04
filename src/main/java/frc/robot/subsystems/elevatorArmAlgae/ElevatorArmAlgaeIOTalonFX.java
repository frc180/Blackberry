package frc.robot.subsystems.elevatorArmAlgae;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorArmAlgaeIOTalonFX implements ElevatorArmAlgaeIO{

    TalonFX motor;
    DigitalInput sensor;
    
    public ElevatorArmAlgaeIOTalonFX() {
        motor = new TalonFX(Constants.ELEVATOR_ARM_ALGAE);
        sensor = new DigitalInput(Constants.ELEVATOR_ARM_ALGAE_SENSOR);
    }

    @Override
    public void run() {
        motor.set(1);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void reverse() {
        motor.set(-1);
    }

    @Override
    public void update(ElevatorArmAlgaeInputs inputs) {
        inputs.hasAlgae = sensor.get();
    }

    
}
