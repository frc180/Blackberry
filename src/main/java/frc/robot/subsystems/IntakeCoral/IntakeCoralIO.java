package frc.robot.subsystems.IntakeCoral;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IntakeCoralIO {

    @Logged
    public class IntakeIOInputs {
        public double voltage = 0;
        public boolean coralSensor = false;
    }

    public void update(IntakeIOInputs inputs);
    
    //to intake a coral
    public void startRollers();

    public void stopRollers();

    public default void simulationPeriodic() {}

    public void runMotorTest();



}
