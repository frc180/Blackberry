package frc.robot.subsystems.IntakeAlgae;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeIO.IntakeAlgaeIOInputs;

@Logged
public class IntakeAlgaeSubsystem extends SubsystemBase {

    public IntakeAlgaeIO io;
    public IntakeAlgaeIOInputs inputs;

    public final Trigger hasAlgae = new Trigger(() -> inputs.hasAlgae);

    public IntakeAlgaeSubsystem() {
        inputs = new IntakeAlgaeIOInputs();
        if (Robot.isReal()) {
            io = new IntakeAlgaeIOSim();
        } else {
            io = new IntakeAlgaeIOSim();
        }
    }

    @Override
    public void periodic() {
        io.update(inputs);
    }

    public Command intake() {
        return this.run(() -> {
            io.startRollers();
            });
    }

    public Command stopIntake() {
        return this.run(() -> {
            io.stopRollers();
        });
    }
}
