package frc.robot.subsystems.elevatorArmAlgae;

public class ElevatorArmAlgaeIOSim implements ElevatorArmAlgaeIO{

    double speed = 0;
    boolean hasCoral;

    public ElevatorArmAlgaeIOSim() {}

    @Override
    public void run() {
        speed = 1;
    }
    
    @Override
    public void stop() {
        speed = 0;
    }

    @Override
    public void reverse() {
        speed = -1;
    }

    @Override
    public void update(ElevatorArmAlgaeInputs inputs) {
        //later
    }
}
