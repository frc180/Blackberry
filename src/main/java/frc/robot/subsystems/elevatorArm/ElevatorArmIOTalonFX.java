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
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeCoral.IntakeCoralSubsystem;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;

public class ElevatorArmIOTalonFX implements ElevatorArmIO {

    
    TalonFX rollerMotor;
    // TODO: add front and back sensors too
    DigitalInput frontSensor, middleSensor, backSensor;
    boolean readyForCoral = false;

    public ElevatorArmIOTalonFX() {
        rollerMotor = new TalonFX(Constants.ELEVATOR_ARM_TALON);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setInverted(false);

        frontSensor = new DigitalInput(Constants.ELEVATOR_ARM_FRONT_SENSOR);
        middleSensor = new DigitalInput(Constants.ELEVATOR_ARM_MIDDLE_SENSOR);
        backSensor = new DigitalInput(Constants.ELEVATOR_ARM_BACK_SENSOR);
    }
    
    @Override
    public void run() {
        rollerMotor.set(1);
    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {
        ElevatorArmPivotSubsystem elevatorArm = RobotContainer.instance.elevatorArmPivot;
        IntakeCoralPivotSubsystem intakeCoralPivot = RobotContainer.instance.intakeCoralPivot;
        IntakeCoralSubsystem intake = RobotContainer.instance.intakeCoral;
        readyForCoral = elevatorArm.isAtReceivingPosition() && intakeCoralPivot.isAtTarget() && intake.hasCoral.getAsBoolean();


        inputs.middleCoralSensor = middleSensor.get();
        inputs.backCoralSensor = backSensor.get();
        inputs.frontCoralSensor = frontSensor.get();
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
