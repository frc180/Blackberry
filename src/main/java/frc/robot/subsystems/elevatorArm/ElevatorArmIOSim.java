package frc.robot.subsystems.elevatorArm;

import frc.robot.RobotContainer;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class ElevatorArmIOSim implements ElevatorArmIO{
 
    double rollerSpeed = 0;
    boolean armHasCoral = false;
    boolean readyForCoral = false;

    public ElevatorArmIOSim() {}

    @Override
    public void run() {
        rollerSpeed = 1;
    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {
        ElevatorArmPivotSubsystem armPivot = RobotContainer.instance.elevatorArmPivot;
        readyForCoral = armPivot.isAtReceivingPosition();
        //armHasCoral = readyForCoral && rollerSpeed > 0;

        if (readyForCoral) {
            run();
            armHasCoral = true;
        }

        SimLogic.hasCoral = false;

        inputs.voltage = Math.abs(rollerSpeed) * 12;
        inputs.coralSensor = armHasCoral;
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
