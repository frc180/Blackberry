package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ClimberIO {

    @Logged
    public class ClimberInputs {
        double jointPosition = 0;
    }

    public void update(ClimberInputs inputs);

    public void setSpeed(double speed);
    
}
