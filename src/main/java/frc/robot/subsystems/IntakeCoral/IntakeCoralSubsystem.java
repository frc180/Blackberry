package frc.robot.subsystems.IntakeCoral;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeCoral.IntakeCoralIO.IntakeIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

@Logged
public class IntakeCoralSubsystem extends SubsystemBase {

    public IntakeCoralIO io;

    public IntakeCoralSubsystem() {
        
        if (Robot.isReal()) {
            io = new IntakeCoralIOSim();
        } else {
            io = new IntakeCoralIOSim();
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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


    
}
