package frc.robot.subsystems.elevatorArmAlgae;

public interface ElevatorArmAlgaeIO {

    public class ElevatorArmAlgaeInputs {
        public boolean aglaeSensor;
    }
    
    public void run();

    public void stop();

    public void reverse();

    public void update(ElevatorArmAlgaeInputs inputs);
}
