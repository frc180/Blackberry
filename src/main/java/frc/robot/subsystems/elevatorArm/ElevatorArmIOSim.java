package frc.robot.subsystems.elevatorArm;

public class ElevatorArmIOSim implements ElevatorArmIO{
 
    double rollerSpeed = 0;
    boolean hasCoral = false;

    public ElevatorArmIOSim() {}

    @Override
    public void run() {
        rollerSpeed = 1;
    }

    @Override
    public void update(ElevatorArmIOInputs inputs) {
        inputs.voltage = rollerSpeed * 12;
        inputs.coralSensor = hasCoral;
    }

    @Override
    public void reverse() {
        rollerSpeed = -1;
    }

    @Override
    public void stop() {
        rollerSpeed = 0;
    }
}
