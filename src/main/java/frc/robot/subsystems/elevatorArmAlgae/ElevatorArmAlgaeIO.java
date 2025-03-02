package frc.robot.subsystems.elevatorArmAlgae;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmAlgaeIO {

    @Logged
    public class ElevatorArmAlgaeInputs {
        public boolean hasAlgae;
        public double speed;
        public double distance;
    }

    public void setSpeed(double speed);
    
    public void stop();

    public void update(ElevatorArmAlgaeInputs inputs);
}
