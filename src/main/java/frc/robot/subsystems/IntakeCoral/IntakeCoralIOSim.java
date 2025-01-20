package frc.robot.subsystems.IntakeCoral;

import edu.wpi.first.math.controller.PIDController;

public class IntakeCoralIOSim implements IntakeCoralIO {
    
    double rollerSpeed;

    public IntakeCoralIOSim() {
    }

    @Override
    public void startRollers() {
        rollerSpeed = 1;
        System.out.println("Coral Intake Roller Speed: " + rollerSpeed);
    }

    @Override
    public void stopRollers() {
        rollerSpeed = 0;
        System.out.println("Coral Intake Roller Speed: " + rollerSpeed);
    }



}
