package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotIO.IntakeAlgaePivotIOInputs;

public class IntakeAlgaePivotSubsystem extends SubsystemBase {
    
    public static final double extend = 100;
    public static final double stow = 0;

    public IntakeAlgaePivotIO io;
    private IntakeAlgaePivotIOInputs inputs;

    public IntakeAlgaePivotSubsystem() {
        inputs = new IntakeAlgaePivotIOInputs();

        if(Robot.isReal()) {
            io = new IntakeAlgaePivotIOSim();
        } else {
            io = new IntakeAlgaePivotIOSim();
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);
    }

    public Command extend() {
        return this.run(() -> {
            io.setPosition(100);
        });
    }

    public Command stow() {
        return this.run (() -> {
            io.setPosition(0);
        });
    }
}
