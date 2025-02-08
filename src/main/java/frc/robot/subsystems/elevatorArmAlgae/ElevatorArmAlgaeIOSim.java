package frc.robot.subsystems.elevatorArmAlgae;

import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class ElevatorArmAlgaeIOSim implements ElevatorArmAlgaeIO{

    double speed = 0;
    boolean fromReef;
    boolean fromIntake;
    boolean readyForAlgae;
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
        IntakeAlgaeSubsystem algaeIntake = RobotContainer.instance.intakeAlgae;

        fromReef = elevator.isElevatorInReefAlgaePosition() && elevatorArmPivot.isElevatorArmInScoringPosition();
        fromIntake = algaeIntake.hasAlgae.getAsBoolean() && elevatorArmPivot.isAtReceivingPosition();
        readyForAlgae = fromReef || fromIntake;
        //hasAlgae = readyForAlgae && speed != 0;
        if (readyForAlgae && speed != 0) {
            hasAlgae = true;
            algaeIntake.inputs.hasAlgae = false;
        }
        
        if (speed < 0) hasAlgae = false;


        // inputs.algaeSensor = hasAlgae;
        inputs.hasAlgae = SimLogic.armHasAlgae || hasAlgae;
        inputs.speed = speed;
    }

    @Override
    public void runMotorTest() {
        System.out.println("elevator arm algae test");

    }
}
