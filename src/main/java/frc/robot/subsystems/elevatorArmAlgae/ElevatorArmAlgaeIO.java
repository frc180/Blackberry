package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmAlgaeIO {

    @Logged
    public class ElevatorArmAlgaeInputs {
        public boolean hasAlgae;
    }
    
    public void run();

    public void stop();

    public void reverse();

    public void update(ElevatorArmAlgaeInputs inputs);
}
