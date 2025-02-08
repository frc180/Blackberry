// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

  // Presets in meters, with 0 being the bottom of the elevator
  public static final double NET = 1.47;
  public static final double L4 = 1.4;
  public static final double L3 = 1;
  public static final double L2 = 0.6;
  public static final double L1 = 0.25;

  public ElevatorIO io;
  private ElevatorIOInputs inputs;

  private double targetPosition = 0;

  public Trigger elevatorInPosition = new Trigger(() -> isElevatorInPosition());
  public Trigger elevatorInScoringPosition = new Trigger(() -> isElevatorInScoringPosition());

  public ElevatorSubsystem() {
    inputs = new ElevatorIOInputs();

    if (Robot.isReal()) {
      io = new ElevatorIOSim();
    } else {
      io = new ElevatorIOTalonFX();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(inputs);
    SimVisuals.setElevatorHeight(inputs.position);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation.
    // Runs before periodic()
    io.simulationPeriodic();
  }

  public double getPositionMeters() {
    return inputs.position;
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public Command runSpeed(double speed) {
    return this.runEnd(() -> {
      io.setPower(speed);
    },
    () -> {
      io.setPower(0);
    });
  }

  public Command setPosition(double encoderPosition) {
    return this.run(() -> {
      setPositionDirect(encoderPosition);
    });
  }

  public Command test() {
    return this.run(() -> {
      io.runMotorTest();
    });
  }

  public Command stop() {
    return this.run(() -> {
      io.stopMotor();
    });
  }

  public boolean isElevatorInPosition() {
    if (Math.abs(targetPosition - inputs.position) <= 0.025) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isElevatorInScoringPosition(){
    if ((targetPosition  != 0) && (Math.abs(targetPosition - inputs.position) <= 0.025)){
      return true;
    } else {
      return false;
    }
  }

  public boolean isElevatorInReefAlgaePosition() {
    if (targetPosition == L2 || targetPosition == L3) {
      return true;
    } else {
      return false;
    }
  }

  public void setPositionDirect(double encoderPosition) {
    io.setPosition(encoderPosition);
    targetPosition = encoderPosition;
  }
  
  public double levelToPosition(int level) {
    switch (level) {
      case 0:
        return 0;
      case 1:
        return L1;
      case 2:
        return L2;
      case 3:
        return L3;
      case 4:
        return L4;
      case 5:
        return NET;
      default:
        return 0;
    }
  }
}
