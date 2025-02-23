package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeAlgaePivotIO {

    @Logged
    public class IntakeAlgaePivotIOInputs {
        double position;
        double voltage;
        double target;
        double absoluteEncoderPosition;
    }

    public void update(IntakeAlgaePivotIOInputs inputs);
    
    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}

    public void runMotorTest();

    public void setSpeed(double speed);

    public void stopMotor();
}
