package frc.robot.subsystems.elevatorArm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeCoral.IntakeCoralSubsystem;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;

public class ElevatorArmIOTalonFX implements ElevatorArmIO {

    TalonFX rollerMotor;
    DigitalInput frontSensor, middleSensor, backSensor;

    VoltageOut voltageControl;
    boolean readyForCoral = false;

    public ElevatorArmIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        rollerMotor = new TalonFX(Constants.ELEVATOR_ARM_TALON);
        rollerMotor.getConfigurator().apply(config);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);

        frontSensor = new DigitalInput(Constants.ELEVATOR_ARM_FRONT_SENSOR);
        middleSensor = new DigitalInput(Constants.ELEVATOR_ARM_MIDDLE_SENSOR);
        backSensor = new DigitalInput(Constants.ELEVATOR_ARM_BACK_SENSOR);

        voltageControl = new VoltageOut(0);
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
        inputs.voltage = rollerMotor.getMotorVoltage(true).getValueAsDouble();
    }

    @Override
    public void run() {
        setSpeed(1);
    }

    @Override
    public void reverse() {
        setSpeed(-1);
    }

    @Override
    public void stop() {
        setSpeed(0);
    }

    private void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    private void setVoltage(double volts) {
        rollerMotor.setControl(voltageControl.withOutput(volts));
    }
}
