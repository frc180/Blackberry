package frc.robot.subsystems.elevatorArmAlgae;

import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class ElevatorArmAlgaeIOSim implements ElevatorArmAlgaeIO{

    double speed = 0;
    boolean hasAlgae;
    boolean readyForScore;

    public ElevatorArmAlgaeIOSim() {}

    @Override
    public void run() {
        speed = 1;
    }
    
    @Override
    public void stop() {
        speed = 0;
    }

    @Override
    public void reverse() {
        speed = -1;
    }

    @Override
    public void update(ElevatorArmAlgaeInputs inputs) {
        //if the elevator is at L2 or L3 and the rollers are running, then we can lie to the simulation and say we have an algae
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        ElevatorArmPivotSubsystem elevatorArmPivot = RobotContainer.instance.elevatorArmPivot;
        hasAlgae = ((elevator.getPositionMeters() == elevator.L2) || (elevator.getPositionMeters() == elevator.L3) || (elevatorArmPivot.isAtReceivingPosition())) && (speed > 0);

        // inputs.algaeSensor = hasAlgae;
        inputs.hasAlgae = SimLogic.armHasAlgae;
    }
}
