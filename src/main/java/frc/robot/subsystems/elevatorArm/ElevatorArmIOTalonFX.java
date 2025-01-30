package frc.robot.subsystems.elevatorArm;

import java.util.List;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorArmIOTalonFX implements ElevatorArmIO {

    
    TalonFX rollerMotor;

    public ElevatorArmIOTalonFX() {
        rollerMotor = new TalonFX(Constants.ELEVATOR_ARM_TALON);
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.getConfigurator().apply(configuration);
    }
    
    @Override
    public void run() {

    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {

    }

    @Override
    public void reverse() {

    }

    @Override
    public void stop() {

    }
    
}
