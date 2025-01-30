package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.math.controller.PIDController;

public class IntakeAlgaePivotIOSim implements IntakeAlgaePivotIO {
    
    PIDController pid;
    boolean usingPID = false;
    double position = 0;
    double speed = 0;
    double target = 0;

    public IntakeAlgaePivotIOSim() {
        pid = new PIDController(0.1,  0, 0);
    }

    @Override
    public void update(IntakeAlgaePivotIOInputs inputs) {
        if (usingPID) {
            speed = pid.calculate(position, target);
            speed = Math.max(-1, Math.min(1, speed));
        }

        inputs.position = position;
        inputs.voltage = speed * 12;
        inputs.target = target;
    }
    
    @Override
    public void setPosition(double encoderPosition) {
        target = encoderPosition;
        usingPID = true;
        //System.out.println("Current algae intake position: " + position);
    }

    @Override
    public void simulationPeriodic() {
        position += speed * 8;
    }
}
