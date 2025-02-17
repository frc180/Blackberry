package frc.robot.subsystems.elevatorArmPivot;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmPivotIO {
    
    @Logged
    public class ElevatorArmPivotIOInputs {
        StatusCode signalStatus;
        double position;
        double voltage;
        double target;
    }
    
    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}

    public void update(ElevatorArmPivotIOInputs inputs);

    public void setSpeed(double speed);

    public void stopMotor();

    
}
