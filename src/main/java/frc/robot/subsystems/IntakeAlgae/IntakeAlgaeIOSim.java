package frc.robot.subsystems.IntakeAlgae;

import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotSubsystem;

public class IntakeAlgaeIOSim implements IntakeAlgaeIO {

    double rollerSpeed;
    boolean hasAlgae;

    public IntakeAlgaeIOSim() {}

    @Override   
    public void startRollers() {
        rollerSpeed = 1;
        System.out.println("Algae Intake Roller Speed: " + rollerSpeed);
    }

    @Override
    public void stopRollers() {
        rollerSpeed = 0;
        System.out.println("Algae Intake Roller Speed: " + rollerSpeed);
    }

    @Override
    public void update(IntakeAlgaeIOInputs inputs) {
        IntakeAlgaePivotSubsystem intakePivot = RobotContainer.instance.intakeAlgaePivot;
        hasAlgae = (intakePivot.getPositionDegrees() == intakePivot.extend) && (rollerSpeed > 0);

        inputs.hasAlgae = hasAlgae;
    }
    
}
