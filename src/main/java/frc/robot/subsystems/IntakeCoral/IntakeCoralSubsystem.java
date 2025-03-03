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
            // io = new IntakeCoralIOSpark();
            // io = new IntakeCoralIOSim();
        } else {
            io = new IntakeCoralIOSim();
        }

        hasCoral = new Trigger(() -> inputs.coralSensor);
        doneIntaking = hasCoral.and(() -> inputs.voltage > 0);
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command intake() {
        return run(() -> io.setSpeed(1));
    }

    public Command stopIntake() {
        return run(() -> io.setSpeed(0));
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> io.setSpeed(speed),
            () -> io.setSpeed(0)
        );
    }
}
