package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeAlgaePivotIO {

    @Logged
    public class IntakeAlgaePivotIOInputs {
        public double position;
        public double voltage;
        public double target;
    }

    public void update(IntakeAlgaePivotIOInputs inputs);
    
    public void setPosition(double encoderPosition);

    public default void simulationPeriodic() {}
}
