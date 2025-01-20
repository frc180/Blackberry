package frc.robot.subsystems.IntakeCoralPivot;

import edu.wpi.first.math.controller.PIDController;

public class IntakeCoralPivotIOSim implements IntakeCoralPivotIO{

    double position = 0;
    double speed = 0;
    double target = 0;

    PIDController pid;
    boolean usingPID = false;

    public IntakeCoralPivotIOSim() {
        pid = new PIDController(0.01, 0, 0);
    }

    @Override
    public void setIntakePosition(double encoderPosition){
        target = encoderPosition;
        usingPID = true;
        //System.out.println("Target Position: " + target);
        System.out.println("Current Intake Position: " + position);
    }

    @Override
    public void update(IntakeCoralPivotIOInputs inputs) {
        if (usingPID) {
            speed = pid.calculate(position, target);
            speed = Math.max(-1, Math.min(1, speed));
        }

        inputs.position = position;
        inputs.voltage = speed * 12;
        inputs.target = target;

    }

    @Override
    public void simulationPeriodic() {
        position += speed * 8;
    }
    
}
