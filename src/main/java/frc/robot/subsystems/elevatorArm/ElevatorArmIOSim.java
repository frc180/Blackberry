package frc.robot.subsystems.elevatorArm;

import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class ElevatorArmIOSim implements ElevatorArmIO{
 
    double rollerSpeed = 0;
    boolean readyForCoral = false;

    public ElevatorArmIOSim() {}

    @Override
    public void run() {
        rollerSpeed = 1;
    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {
        ElevatorArmPivotSubsystem armPivot = RobotContainer.instance.elevatorArmPivot;
        IntakeCoralPivotSubsystem intakeCoralPivot = RobotContainer.instance.intakeCoralPivot;
        readyForCoral = SimLogic.intakeHasCoral && armPivot.isAtReceivingPosition() && intakeCoralPivot.isAtTarget();

        if (readyForCoral && rollerSpeed > 0) {
            SimLogic.armHasCoral = true;
            SimLogic.intakeHasCoral = false;
        }

        inputs.voltage = Math.abs(rollerSpeed) * 12;
        inputs.frontCoralSensor = SimLogic.armHasCoral;
        inputs.middleCoralSensor = SimLogic.armHasCoral;
        inputs.backCoralSensor = SimLogic.armHasCoral;
    }

    @Override
    public void reverse() {
        rollerSpeed = -1;
    }

    @Override
    public void stop() {
        rollerSpeed = 0;
    }

    @Override
    public void runMotorTest() {
        System.out.println("elevator arm test");

    }
}
