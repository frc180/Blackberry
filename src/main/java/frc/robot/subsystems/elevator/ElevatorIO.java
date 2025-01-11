package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    public void update();
    
    public void setPower(double power);

    public void setPosition(double encoderPosition);

}
