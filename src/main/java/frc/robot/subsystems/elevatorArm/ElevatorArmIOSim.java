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
        readyForCoral = SimLogic.intakeHasCoral && armPivot.isAtReceivingPosition();// && intakeCoralPivot.isAtTarget();

        boolean armHasCoralPrevious = SimLogic.armHasCoral;

        if (readyForCoral && rollerSpeed > 0) {
            SimLogic.armHasCoral = true;
            SimLogic.intakeHasCoral = false;
        }

        if (SimLogic.armHasCoral) {
            if (!armHasCoralPrevious) {
                // Reset simulated coral position to start if we just got coral
                SimLogic.armCoralPosition = 0;
            } else if (SimLogic.armCoralPosition == -1) {
                // Preloaded corral
                SimLogic.armCoralPosition = SimLogic.CORAL_LENGTH;
            } else {
                SimLogic.armCoralPosition += rollerSpeed * 0.2;
            }
        }

        boolean backCoral = coralAtSensor(0);
        boolean middleCoral = coralAtSensor(SimLogic.CORAL_LENGTH * .7);
        boolean frontCoral = coralAtSensor((SimLogic.CORAL_LENGTH * .7) * 2);

        // Coral has passed through the entire arm, score it
        if (SimLogic.armHasCoral && !backCoral && !middleCoral && !frontCoral) {
            SimLogic.armHasCoral = false;
            SimLogic.scoreCoral();
        }

        inputs.voltage = Math.abs(rollerSpeed) * 12;
        inputs.backCoralSensor = backCoral;
        inputs.middleCoralSensor = middleCoral;
        inputs.frontCoralSensor = frontCoral;
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
    public void setSpeed(double speed) {
        rollerSpeed = speed;
    }

    @Override
    public void runMotorTest() {
        System.out.println("elevator arm test");

    }

    private boolean coralAtSensor(double sensorPosition) {
        if (!SimLogic.armHasCoral) {
            return false;
        }

        double coralFront = SimLogic.armCoralPosition;
        double coralBack = SimLogic.armCoralPosition - SimLogic.CORAL_LENGTH;
        return sensorPosition >= coralBack && sensorPosition <= coralFront;
    }
}
