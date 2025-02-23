package frc.robot.subsystems.elevatorArmAlgae;

import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class ElevatorArmAlgaeIOSim implements ElevatorArmAlgaeIO{

    double speed = 0;
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
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        ElevatorArmPivotSubsystem elevatorArmPivot = RobotContainer.instance.elevatorArmPivot;
        IntakeAlgaeSubsystem algaeIntake = RobotContainer.instance.intakeAlgae;

        boolean fromReef = elevator.isElevatorInReefAlgaePosition() && elevatorArmPivot.isElevatorArmInScoringPosition();
        boolean fromIntake = algaeIntake.hasAlgae.getAsBoolean() && elevatorArmPivot.isAtReceivingPosition();

        if (SimLogic.armHasAlgae) {
            if (speed < 0) {
                if (elevatorArmPivot.getTargetPosition() == ElevatorArmPivotSubsystem.netScore) {
                    SimLogic.netAlgae(true);
                } else if (elevatorArmPivot.getTargetPosition() == ElevatorArmPivotSubsystem.netScoreBackwards) {
                    SimLogic.netAlgae(false);
                } else {
                    SimLogic.intakeHasAlgae = true;
                }
                SimLogic.armHasAlgae = false;
            }
        } else {
            if (fromReef) { // TODO: && speed > 0?
                int tag = drivetrain.getTargetPoseTag();
                if (tag != -1 && Field.hasReefAlgae(tag) && RobotContainer.instance.driverRightReef.getAsBoolean()) {
                    SimLogic.armHasAlgae = true;
                    Field.removeReefAlgae(tag);
                }
            }

            if (fromIntake && speed > 0) {
                SimLogic.armHasAlgae = true;
                SimLogic.intakeHasAlgae = false;
            }
        }

        // inputs.algaeSensor = hasAlgae;
        inputs.hasAlgae = SimLogic.armHasAlgae;
    }

    @Override
    public void setSpeed(double speed) {
        // System.out.println("elevator arm algae test");
    }
}
