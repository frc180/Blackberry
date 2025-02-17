package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorIO {

    @Logged
    public class ElevatorIOInputs {
        StatusCode signalStatus = StatusCode.OK;
        double position;
        double velocity;
        double voltage;
        double target;
        boolean bottomLimit = false;
    }

    public void update(ElevatorIOInputs inputs);
    
    public void setPower(double power);

    public void setVoltage(double volts);

    public void setPosition(double encoderPosition);

    public void stopMotor();

    public void zero();

    public default void simulationPeriodic() {}

}
