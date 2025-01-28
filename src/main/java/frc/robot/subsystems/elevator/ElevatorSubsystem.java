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
      // There is no elevator on the test robot, so we fake it
      io = new ElevatorIOSim();
    } else {
      // TalonFX supports realistic physics simulation
      io = new ElevatorIOSim();
    }
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

  public boolean isElevatorInPosition() {
    if (Math.abs(targetPosition - inputs.position) <= 0.025) {
      return true;
    } else {
      return false;
    }
  }

  // TODO: There's a bug here! Can you find it? Compare what this one is doing to isElevatorInPosition()
  // (Hint: We should be checking if we're in scoring position, not just targeting scoring position)
  public boolean isElevatorInScoringPosition(){
    if (targetPosition  != 0){
      return true;
    } else {
      return false;
    }
  }

  public void setPositionDirect(double encoderPosition) {
    io.setPosition(encoderPosition);
    targetPosition = encoderPosition;
  }
}
