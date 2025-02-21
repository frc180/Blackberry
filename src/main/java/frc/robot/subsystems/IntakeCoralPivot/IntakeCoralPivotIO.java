package frc.robot.subsystems.IntakeCoralPivot;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeCoralPivotIO {

    @Logged
    public class IntakeCoralPivotIOInputs {
        double position;
        double voltage;
        double target;
    }
    
    //to stow and extend
    public void setIntakePosition(double position);

    public default void simulationPeriodic() {}

    public void update(IntakeCoralPivotIOInputs inputs);

    public void runMotorTest();

    public void stopMotor();
}
