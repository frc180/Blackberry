package frc.robot.subsystems.elevatorArm;

import frc.robot.RobotContainer;
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
        // TODO: eventually add logic that ensures the intake is also at the right position to pass off coral
        ElevatorArmPivotSubsystem armPivot = RobotContainer.instance.elevatorArmPivot;
        readyForCoral = SimLogic.intakeHasCoral && armPivot.isAtReceivingPosition();

        if (readyForCoral && rollerSpeed > 0) {
            SimLogic.armHasCoral = true;
            SimLogic.intakeHasCoral = false;
        }

        inputs.voltage = Math.abs(rollerSpeed) * 12;
        inputs.coralSensor = SimLogic.armHasCoral;
    }

    @Override
    public void reverse() {
        rollerSpeed = -1;
    }

    @Override
    public void stop() {
        rollerSpeed = 0;
    }
}
