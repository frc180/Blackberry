package frc.robot.subsystems.elevatorArm;

import java.util.List;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorArmIOTalonFX implements ElevatorArmIO {

    
    TalonFX rollerMotor;
    DigitalInput armSensor;

    public ElevatorArmIOTalonFX() {
        rollerMotor = new TalonFX(Constants.ELEVATOR_ARM_TALON);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setInverted(false);
    }
    
    @Override
    public void run() {
        rollerMotor.set(1);
    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {
        inputs.coralSensor = armSensor.get();
        inputs.voltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void reverse() {
        rollerMotor.set(-1);
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
    }
    
}
