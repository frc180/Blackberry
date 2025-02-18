package frc.robot.subsystems.elevatorArmPivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorArmPivotIOTalonFX implements ElevatorArmPivotIO {
    final double PIVOT_GEARING = 128;
    final double radToRotations = 1 / (2 * Math.PI);

    // temporary
    final double gearingFix = 128 / PIVOT_GEARING; // old gearing / new gearing

    TalonFX armPivotMotor;
    MotionMagicVoltage motionMagic;
    VoltageOut voltageControl;

    // Status signals
    StatusSignal<Angle> positionSignal;
    StatusSignal<Voltage> voltageSignal;
    StatusSignal<Double> targetSignal;
    StatusSignal<AngularVelocity> velocitySignal;
    StatusSignal<Current> currentSignal;

    // Simulation-only variables
    TalonFXSimState armPivotMotorSim;
    SingleJointedArmSim armSim;

    public ElevatorArmPivotIOTalonFX() {
        armPivotMotor = new TalonFX(Constants.ELEVATOR_ARM_PIVOT_TALON, Constants.CANIVORE);
        TalonFXConfiguration config = new TalonFXConfiguration();
        if (Robot.isReal()) {
            config.Slot0.kP = 150 * gearingFix;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 20 * gearingFix;
            config.Slot0.kA = 0;
            config.MotionMagic.MotionMagicCruiseVelocity = 2.6 * gearingFix;
            config.MotionMagic.MotionMagicAcceleration = 2 * gearingFix;
        } else {
            config.Slot0.kP = 50 * gearingFix;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0.521 * gearingFix;
            config.Slot0.kV = 10 * gearingFix;
            config.Slot0.kA = 0;
            config.MotionMagic.MotionMagicCruiseVelocity = 999;
            config.MotionMagic.MotionMagicAcceleration = 3 * gearingFix;
        }
        config.Feedback.SensorToMechanismRatio = PIVOT_GEARING;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotionMagic.MotionMagicJerk = 0;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorArmPivotSubsystem.FORWARD_LIMIT.in(Rotations);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorArmPivotSubsystem.REVERSE_LIMIT.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        armPivotMotor.getConfigurator().apply(config);
        armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        armPivotMotor.setPosition(ElevatorArmPivotSubsystem.HARD_STOP_OFFSET);

        motionMagic = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);

        positionSignal = armPivotMotor.getPosition();
        voltageSignal = armPivotMotor.getMotorVoltage();
        targetSignal = armPivotMotor.getClosedLoopReference();
        velocitySignal = armPivotMotor.getVelocity();
        currentSignal = armPivotMotor.getSupplyCurrent();

        // Everything below this line is for simulation only
        if (Robot.isReal()) return;

        armPivotMotorSim = armPivotMotor.getSimState();

        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            PIVOT_GEARING,
            1,
            0.4572,
            -Math.PI,
            Math.PI,
            true,
            ElevatorArmPivotSubsystem.HARD_STOP_OFFSET.in(Radians)
        );
    }

    @Override
    public void update(ElevatorArmPivotIOInputs inputs) {
        inputs.signalStatus = BaseStatusSignal.refreshAll(positionSignal, voltageSignal, targetSignal, velocitySignal, currentSignal);
        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();
        inputs.velocity = velocitySignal.getValueAsDouble();
        inputs.current = currentSignal.getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(armPivotMotorSim.getMotorVoltage());
        armSim.update(0.020);

        armPivotMotorSim.setRawRotorPosition(armSim.getAngleRads() * radToRotations * PIVOT_GEARING);
        armPivotMotorSim.setRotorVelocity(armSim.getVelocityRadPerSec() * radToRotations * PIVOT_GEARING);
        armPivotMotorSim.setSupplyVoltage(12);
    }

    @Override
    public void setSpeed(double speed) {
        armPivotMotor.setControl(voltageControl.withOutput(speed * 12));
    }

    @Override
    public void stopMotor() {
        armPivotMotor.stopMotor();
    }

    @Override
    public void setPosition(double encoderPosition) {
        armPivotMotor.setControl(motionMagic.withPosition(encoderPosition));
    }

    @Override
    public void zero(double rotations) {
        armPivotMotor.setPosition(rotations);
    }
}
