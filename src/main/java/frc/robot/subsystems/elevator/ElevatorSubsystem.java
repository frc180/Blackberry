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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    // Distance presets, with 0 being the bottom of the elevator
    public static final Distance L1 = Inches.of(2.5);
    public static final Distance L2 = Meters.of(0.302);
    public static final Distance L3 = L2.plus(Inches.of(16));
    public static final Distance L4 = Meters.of(1.4).plus(Inches.of(1.5));
    public static final Distance NET = Meters.of(1.47); // may be able to just be L4
    public static final Distance STOW = Centimeters.of(0);
    public static final Distance ZERO = Meters.of(0);

    protected static final double SOFT_UPPER_LIMIT = Meters.of(1.48).in(Meters);
    protected static final double SOFT_LOWER_LIMIT = 0;
    private static final double IN_POSITION_METERS = Inches.of(1).in(Meters); // TODO: lower tolerance?
    private static final double STOW_INTERMEDIATE = Inches.of(2).in(Meters);

    private ElevatorIO io;
    private ElevatorIOInputs inputs;
    private Alert notHomedAlert = new Alert("Elevator not homed!", AlertType.kWarning);

    private Distance targetPosition = ZERO;
    private boolean hasHomed = false;

    private Trigger atLowerLimitDebounced = new Trigger(this::isAtLowerLimit).debounce(0.2);

    @NotLogged
    public Trigger elevatorInPosition = new Trigger(this::isElevatorInPosition);
    @NotLogged
    public Trigger elevatorInScoringPosition = new Trigger(this::isElevatorInScoringPosition);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(1), // Ramp rate
                    Volts.of(1), // Dynamic step voltage
                    null, // Use default timeout (10 s)
                    state -> SignalLogger.writeString("ElevatorSysId_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> io.setVoltage(output.in(Volts)),
                    null,
                    this));

    public ElevatorSubsystem() {
        inputs = new ElevatorIOInputs();
        io = new ElevatorIOTalonFX();
        io.zero();
    }

    @Override
    public void periodic() {
        io.update(inputs);
        SimVisuals.setElevatorHeight(inputs.position);

        // To be considered homed, the elevator must be at the lower limit and the motor
        // position must be within 0.01 meters of 0
        if (isAtLowerLimit() && Math.abs(inputs.position) <= 0.01) {
            hasHomed = true;
        }
        notHomedAlert.set(!hasHomed);

        softStowLogic();
    }

    @Override
    public void simulationPeriodic() {
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

    @NotLogged
    public boolean isHomed() {
        return hasHomed;
    }

    public Command home() {
        return runOnce(() -> hasHomed = false)
                .andThen(runSpeed(-0.03).until(atLowerLimitDebounced))
                .andThen(runOnce(() -> io.zero()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command stow() {
        return setPosition(STOW);
    }

    public Command setPosition(Distance position) {
        return run(() -> setPositionDirect(position));
    }

    public Command runSpeed(double speed) {
        return runEnd(
                () -> io.setPower(speed),
                () -> io.stopMotor());
    }

    public Command stop() {
        return run(() -> io.stopMotor());
    }

    public Command sysIdQuasi(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction)
                .until(sysIdEnd(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction)
                .until(sysIdEnd(direction));
    }

    private BooleanSupplier sysIdEnd(SysIdRoutine.Direction direction) {
        return direction == SysIdRoutine.Direction.kReverse ? this::isAtSoftLowerLimit : this::isAtSoftUpperLimit;
    }

    public boolean isAtSoftUpperLimit() {
        return inputs.position >= SOFT_UPPER_LIMIT;
    }

    public boolean isAtSoftLowerLimit() {
        return inputs.position <= SOFT_LOWER_LIMIT;
    }

    public boolean isAtLowerLimit() {
        return inputs.bottomLimit;
    }

    public boolean isElevatorInPosition() {
        return Math.abs(getTargetErrorMeters()) <= IN_POSITION_METERS;
    }

    public boolean isElevatorInScoringPosition() {
        return isElevatorInPosition() && targetPosition != ZERO && targetPosition != STOW;
    }

    public boolean isTargetingReefAlgaePosition() {
        return targetPosition == L2 || targetPosition == L3;
    }

    public boolean isElevatorInReefAlgaePosition() {
        return isElevatorInPosition() && isTargetingReefAlgaePosition();
    }

    public double getTargetErrorMeters() {
        return targetPosition.in(Meters) - inputs.position;
    }

    /**
     * Handles special soft-stow logic for stowing the elevator safely.
     * 
     * @return True if the soft stow logic was applied, false otherwise.
     */
    private boolean softStowLogic() {
        if (targetPosition != STOW) return false;

        if (atLowerLimitDebounced.getAsBoolean()) {
            io.stopMotor();
        } else if (inputs.position > STOW_INTERMEDIATE + IN_POSITION_METERS) {
            io.setPosition(STOW_INTERMEDIATE);
        } else {
            io.setPower(-0.05);
        }
        return true;
    }

    public void setPositionDirect(Distance position) {
        targetPosition = position;
        if (!softStowLogic()) {
            io.setPosition(position.in(Meters));
        }
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
