package frc.robot.subsystems.IntakeAlgae;

public class IntakeAlgaeIOSim implements IntakeAlgaeIO {

    double rollerSpeed;

    public IntakeAlgaeIOSim() {
        
    }

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
    
}
