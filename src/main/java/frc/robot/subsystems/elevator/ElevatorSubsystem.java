// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

  public static final int L4 = 450;
  public static final int L3 = 350;
  public static final int L2 = 200;
  public static final int L1 = 100;

  public ElevatorIO io;
  private ElevatorIOInputs inputs;

  private double targetPosition = 0;

  public ElevatorSubsystem() {
    inputs = new ElevatorIOInputs();

    io = new ElevatorIOTalonFX();
    // if (Robot.isReal()) {
    //   io = new ElevatorIOTalonFX();
    // } else {
    //   io = new ElevatorIOSim();
    // }
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
      targetPosition = encoderPosition;
    });
  }
}
