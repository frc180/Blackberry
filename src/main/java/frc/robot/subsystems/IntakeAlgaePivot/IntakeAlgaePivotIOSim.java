package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.math.controller.PIDController;

public class IntakeAlgaePivotIOSim implements IntakeAlgaePivotIO {
    
    PIDController pid;
    boolean usingPID = false;
    double position = IntakeAlgaePivotSubsystem.stow;
    double speed = 0;
    double target = 0;

    public IntakeAlgaePivotIOSim() {
        pid = new PIDController(0.05,  0, 0);
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
    }

    @Override
    public void simulationPeriodic() {
        position += speed * 4;
    }

    @Override
    public void runMotorTest() {
        System.out.println("intake algae pivot test");

    }

    @Override
    public void stopMotor() {
        speed = 0;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
