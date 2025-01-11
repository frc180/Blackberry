package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSim implements ElevatorIO {

    PIDController pid;
    double position = 0;
    double target = 0;

    public ElevatorIOSim() {
        pid = new PIDController(5, 0, 0);
    }

    @Override
    public void setPower(double power) {
        System.out.println("Elevator is running at power " + power);
    }

    @Override
    public void setPosition(double encoderPosition) {
        target = encoderPosition;
    }

    @Override
    public void update() {
        double speed = pid.calculate(position, target);
        position += speed / 100;
        System.out.println(position);
    }
}
