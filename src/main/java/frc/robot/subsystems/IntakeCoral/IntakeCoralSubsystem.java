package frc.robot.subsystems.IntakeCoral;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoral.IntakeCoralIO.IntakeIOInputs;

@Logged
public class IntakeCoralSubsystem extends SubsystemBase {

    private IntakeCoralIO io;
    private IntakeIOInputs inputs;

    public final Trigger hasCoral;
    public final Trigger doneIntaking;

    public IntakeCoralSubsystem() {
        inputs = new IntakeIOInputs();
        if (Robot.isReal()) {
            io = new IntakeCoralIOTalonFXS();
        } else {
            io = new IntakeCoralIOSim();
        }

        hasCoral = new Trigger(() -> inputs.coralSensor);
        doneIntaking = hasCoral.and(() -> inputs.voltage > 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation.
        // Runs before periodic()
        io.simulationPeriodic();
    }

    public Command intake() {
        return this.run (() -> {
            io.startRollers();
        });
    }

    public Command stopIntake() {
        return this.run(() -> {
            io.stopRollers();
        });
    }

    public Command test() {
        return this.run(() -> {
          io.runMotorTest();
        });
    }
    
}
