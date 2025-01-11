// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {

  public static final int L2 = 400;

  public ElevatorIO io;

  public ElevatorSubsystem() {
    if (Robot.isReal()) {
      // Real robot
      io = new ElevatorIOTalonFX();
    } else {
      // Simulation
      io = new ElevatorIOSim();
    }
  }

  @Override
  public void periodic() {
    io.update();
    // This method will be called once per scheduler run
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
      io.setPosition(encoderPosition);
    });
  }
}
