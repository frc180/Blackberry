package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorIO {

    @Logged
    public class ElevatorIOInputs {
        public double position;
        public double voltage;
        public double target;
    }

    public void update(ElevatorIOInputs inputs);
    
    public void setPower(double power);

    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}
}
