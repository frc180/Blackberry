package frc.robot.subsystems.IntakeCoral;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeCoralIO {

    @Logged
    public class IntakeIOInputs {
        public double position;
        public double voltage;
        public double target;
    }
    
    //to intake a coral
    public void startRollers();

    public void stopRollers();

    public default void simulationPeriodic() {}



}
