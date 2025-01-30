package frc.robot.subsystems.elevatorArmPivot;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorArmPivotIOSim implements ElevatorArmPivotIO{

    double position;
    double speed;
    double target;

    PIDController pid;
    boolean usingPID = false;

    public ElevatorArmPivotIOSim() {
        pid = new PIDController(0.01, 0, 0);
    }

    @Override
    public void setPosition(double encoderPosition) {
        target = encoderPosition;
        usingPID = true;

    }

    @Override
    public void update(ElevatorArmPivotIOInputs inputs) {
        if (usingPID) {
            speed = pid.calculate(position, target);
            speed = Math.max(-1, Math.min(1, speed));

        }

        inputs.position = position;
        inputs.target = target;
        inputs.voltage = speed * 12;
    }
    
    @Override
    public void simulationPeriodic() {
        position += speed * 8;
    }
}
