package frc.robot.subsystems.IntakeAlgaePivot;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeAlgaePivotIO {

    @Logged
    public class IntakeAlgaePivotIOInputs {
        StatusCode signalStatus;
        double position;
        double voltage;
        double target;
        double encoderPosition;
    }

    public void update(IntakeAlgaePivotIOInputs inputs);
    
    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}

    public void runMotorTest();

    public void setSpeed(double speed);

    public void stopMotor();
}
