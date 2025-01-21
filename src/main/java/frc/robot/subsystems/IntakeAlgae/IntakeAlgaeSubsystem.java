package frc.robot.subsystems.IntakeAlgae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeIO;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeIOSim;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeIOTalonFXS;

public class IntakeAlgaeSubsystem extends SubsystemBase {

    public IntakeAlgaeIO io;

    public IntakeAlgaeSubsystem() {

        if (Robot.isReal()) {
            io = new IntakeAlgaeIOTalonFXS();
        } else {
            io = new IntakeAlgaeIOSim();
        }
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
