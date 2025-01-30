package frc.robot.subsystems.elevatorArm;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmIO {

    @Logged
    public class ElevatorArmIOInputs {
        public double voltage = 0;
        public boolean coralSensor = false;
    }

    public void run();

    public void update(ElevatorArmIOInputs inputs);

    public void reverse();

    public void stop();

}
