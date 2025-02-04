package frc.robot.subsystems.IntakeAlgae;

public interface IntakeAlgaeIO {

    public class IntakeAlgaeIOInputs {
        public boolean hasAlgae;
    }

    public void startRollers();

    public void stopRollers();

    public void update(IntakeAlgaeIOInputs inputs);
    
}
