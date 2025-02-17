// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

  // Distance presets, with 0 being the bottom of the elevator
  public static final Distance L1 = Meters.of(0.269);       // not real
  public static final Distance L2 = Meters.of(0.302);       // 3 degrees arm pivot
  public static final Distance L3 = L2.plus(Inches.of(16)); // 3 degrees arm pivot
  public static final Distance L4 = Meters.of(1.4);         // 14 degrees pivot
  public static final Distance NET = Meters.of(1.47);       // may be able to just be L4
  public static final Distance STOW = Centimeters.of(1);
  public static final Distance ZERO = Meters.of(0);

  private static final double IN_POSITION_METERS = Inches.of(1).in(Meters);

  private ElevatorIO io;
  private ElevatorIOInputs inputs;
  private Alert notZeroedAlert = new Alert("Elevator not zeroed!", AlertType.kWarning);

  private Distance targetPosition = ZERO;
  private boolean hasZeroed = false;

  @NotLogged
  public Trigger elevatorInPosition = new Trigger(this::isElevatorInPosition);
  @NotLogged
  public Trigger elevatorInScoringPosition = new Trigger(this::isElevatorInScoringPosition);


  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(1), // Ramp rate
          Volts.of(1), // Dynamic step voltage
          null,        // Use default timeout (10 s)
          state -> SignalLogger.writeString("ElevatorSysId_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          output -> io.setVoltage(output.in(Volts)),
          null,
          this
      )
  );

  public ElevatorSubsystem() {
    inputs = new ElevatorIOInputs();
    io = new ElevatorIOTalonFX();
    // io = new ElevatorIOSim();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(inputs);
    SimVisuals.setElevatorHeight(inputs.position);

    // To be considered zeroed, the elevator must be at the lower limit and the motor position must be within 0.01 meters of 0
    if (isAtLowerLimit() && Math.abs(inputs.position) <= 0.01) {
      hasZeroed = true;
    }
    notZeroedAlert.set(!hasZeroed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation.
    // Runs before periodic()
    io.simulationPeriodic();
  }

  @NotLogged
  public double getPositionMeters() {
    return inputs.position;
  }

  @NotLogged
  public Distance getTargetPosition() {
    return targetPosition;
  }

  public void zero() {
    io.zero();
  }

  @NotLogged
  public boolean hasZeroed() {
    return hasZeroed;
  }

  public Command runSpeed(double speed) {
    return runEnd(
        () -> io.setPower(speed),
        () -> io.stopMotor()
    );
  }

  public Command setPosition(Distance position) {
    return run(() -> setPositionDirect(position));
  }

  public Command home() {
    return runSpeed(-0.1).until(this::isAtLowerLimit)
            .andThen(runOnce(this::zero));
  }

  public Command stop() {
    return run(() -> io.stopMotor());
  }

  public Command sysidQuasi(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction).until(sysIdEnd(direction));
  }

  public Command sysidDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction).until(sysIdEnd(direction));
  }

  public BooleanSupplier sysIdEnd(SysIdRoutine.Direction direction) {
    return direction == SysIdRoutine.Direction.kReverse ? this::isAtSoftLowerLimit : this::isAtSoftUpperLimit;
  }

  public boolean isAtSoftUpperLimit() {
    return inputs.position > NET.in(Meters);
  }

  public boolean isAtSoftLowerLimit() {
    return inputs.position <= 0;
  }

  public boolean isAtLowerLimit() {
    return inputs.bottomLimit;
  }

  public boolean isElevatorInPosition() {
    return Math.abs(targetPosition.in(Meters) - inputs.position) <= IN_POSITION_METERS;
  }

  public boolean isElevatorInScoringPosition(){
    return isElevatorInPosition() && targetPosition != ZERO && targetPosition != STOW;
  }

  public boolean isElevatorInReefAlgaePosition() {  
    return isElevatorInPosition() && (targetPosition == L2 || targetPosition == L3);
  }

  public void setPositionDirect(Distance position) {
    io.setPosition(position.in(Meters));
    targetPosition = position;
  }
  
  public Distance levelToPosition(int level) {
    switch (level) {
      case 0:
        return STOW;
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
        return STOW;
    }
  }
}
