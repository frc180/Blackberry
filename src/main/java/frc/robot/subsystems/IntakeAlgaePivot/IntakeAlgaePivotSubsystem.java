package frc.robot.subsystems.IntakeAlgaePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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

    private double targetPosition = stow;

    public IntakeAlgaePivotSubsystem() {
        inputs = new IntakeAlgaePivotIOInputs();

        io = new IntakeAlgaePivotIOTalonFXS();
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
        return setPosition(extend);
    }

    public Command stow() {
        return setPosition(stow);
    }

    public Command readyClimb() {
        return setPosition(climbReady);
    }

    public Command setPosition(double position) {
        return this.run(() -> {
            io.setPosition(position);
            targetPosition = position;
        });
    }

    public Command test() {
        return this.run(() -> {
          io.runMotorTest();
        });
    }

    public Command stop() {
        return this.run (() -> {
            io.stopMotor();
        });
    }

    public double getPositionDegrees() {
        return inputs.position;
    }

    public double getTargetDegrees() {
        return targetPosition;
    }

    public boolean isAtTarget() {
        return Math.abs(getPositionDegrees() - getTargetDegrees()) <= 3;
    }

    @NotLogged
    public Pose3d getPose() {
        double angle = Units.degreesToRadians(inputs.position);
        return new Pose3d(0, 0.2, 0.2, new Rotation3d(0, angle, Units.degreesToRadians(270)));
    }
}
