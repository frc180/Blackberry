package frc.robot.subsystems.elevatorArm;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmIO {

    @Logged
    public class ElevatorArmIOInputs {
        public double voltage = 0;
        public boolean frontCoralSensor = false;
        public boolean middleCoralSensor = false;
        public boolean backCoralSensor = false;
    }

    public void run();

    public void update(ElevatorArmIOInputs inputs);

    public void reverse();

    public void stop();

    public void runMotorTest();

}
