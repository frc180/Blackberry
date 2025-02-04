package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotIO.IntakeAlgaePivotIOInputs;

@Logged
public class IntakeAlgaePivotSubsystem extends SubsystemBase {
    
    public static final double extend = 55;
    public static final double stow = 90;
    public static final double climbReady = 0; //on the floor

    public IntakeAlgaePivotIO io;
    private IntakeAlgaePivotIOInputs inputs;

    public IntakeAlgaePivotSubsystem() {
        inputs = new IntakeAlgaePivotIOInputs();

        if(Robot.isReal()) {
            io = new IntakeAlgaePivotIOTalonFXS();
        } else {
            io = new IntakeAlgaePivotIOSim();
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.update(inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command extend() {
        return this.run(() -> {
            io.setPosition(extend);
        });
    }

    public Command stow() {
        return this.run (() -> {
            io.setPosition(stow);
        });
    }

    public Command readyClimb() {
        return this.run (() -> {
            io.setPosition(climbReady);
        });
    }

    public double getPositionDegrees() {
        return inputs.position;
    }

    public Pose3d getPose() {
        double angle = Units.degreesToRadians(inputs.position);
        return new Pose3d(0, 0.2, 0.2, new Rotation3d(0, angle, Units.degreesToRadians(270)));
    }
}
