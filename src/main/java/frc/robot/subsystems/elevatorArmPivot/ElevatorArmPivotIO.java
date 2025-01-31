package frc.robot.subsystems.elevatorArmPivot;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface ElevatorArmPivotIO {
    
    @Logged
    public class ElevatorArmPivotIOInputs {
        public double position;
        public double voltage;
        public double target;
    }
    
    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}

    public void update(ElevatorArmPivotIOInputs inputs);

    
}
