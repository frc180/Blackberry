package frc.robot.subsystems.elevatorArmPivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorArm.ElevatorArmIOTalonFX;

public class ElevatorArmPivotIOTalonFX implements ElevatorArmPivotIO {
    final double PIVOT_GEARING = 128;
    final double radToRotations = 1 / (2 * Math.PI);
    final Angle ABSOLUTE_POSITION_OFFSET = Degrees.of(0);


    TalonFX armPivotMotor;
    MotionMagicVoltage motionMagic;
    VoltageOut voltageControl;
    NeutralModeValue neutralMode = null;
    AnalogPotentiometer potentiometer;

    // Status signals
    StatusSignal<Angle> positionSignal;
    StatusSignal<Voltage> voltageSignal;
    StatusSignal<Double> targetSignal;
    StatusSignal<AngularVelocity> velocitySignal;

    // Simulation-only variables
    TalonFXSimState armPivotMotorSim;
    SingleJointedArmSim armSim;

    public ElevatorArmPivotIOTalonFX() {
        armPivotMotor = new TalonFX(Constants.ELEVATOR_ARM_PIVOT_TALON, Constants.CANIVORE);
        TalonFXConfiguration config = new TalonFXConfiguration();
        if (Robot.isReal()) {
            config.Slot0.kP = 220; // was 200
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0.23;
            config.Slot0.kV = 15;
            config.Slot0.kA = 0;
        } else {
            config.Slot0.kP = 100;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 17;
            config.Slot0.kA = 0;
        }
        config.MotionMagic.MotionMagicCruiseVelocity = 2.6;
        config.MotionMagic.MotionMagicAcceleration = 2;
        config.MotionMagic.MotionMagicJerk = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Feedback.SensorToMechanismRatio = PIVOT_GEARING;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorArmPivotSubsystem.FORWARD_LIMIT.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorArmPivotSubsystem.REVERSE_LIMIT.in(Rotations);
        // if (Robot.isReal()) {
        //     config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        //     config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // }

        armPivotMotor.getConfigurator().apply(config);
        setNeutralMode(NeutralModeValue.Coast);

        motionMagic = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);

        potentiometer = new AnalogPotentiometer(
            Constants.ANALOG_ELEVATOR_ARM_PIVOT_POTENTIOMETER,
            1,
            ABSOLUTE_POSITION_OFFSET.in(Rotations)
        );

        positionSignal = trackSignal(armPivotMotor.getPosition());
        voltageSignal = trackSignal(armPivotMotor.getMotorVoltage());
        targetSignal = trackSignal(armPivotMotor.getClosedLoopReference());
        velocitySignal = trackSignal(armPivotMotor.getVelocity());

        // Everything below this line is for simulation only
        if (Robot.isReal()) return;

        armPivotMotor.setPosition(ElevatorArmPivotSubsystem.HARD_STOP_OFFSET);
        armPivotMotorSim = armPivotMotor.getSimState();

        double hardStopRadians = ElevatorArmPivotSubsystem.HARD_STOP_OFFSET.in(Radians);
        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            PIVOT_GEARING,
            1,
            0.4572,
            -Math.PI,
            hardStopRadians,
            true,
            hardStopRadians
        );
    }

    @Override
    public void update(ElevatorArmPivotIOInputs inputs) {
        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();
        inputs.velocity = velocitySignal.getValueAsDouble();
        if (Robot.isReal()) {
            inputs.absolutePosition = potentiometer.get();
            // Special case - this signal comes from a CANdi managed by ElevatorArmIOTalonFX, so we need to read it 
            // from there instead of by creating the CANdi and signal objects ourselves here
            StatusSignal<Boolean> signal = ElevatorArmIOTalonFX.HARD_STOP_SIGNAL;
            inputs.hardStop = signal != null && signal.getValueAsDouble() == 1;
        } else {
            inputs.absolutePosition = 30 + (Units.radiansToRotations(armSim.getAngleRads()) * 0.25);
            double armDegrees = Units.radiansToDegrees(armSim.getAngleRads());
            inputs.hardStop = Math.abs(armDegrees - ElevatorArmPivotSubsystem.HARD_STOP_OFFSET.in(Degrees)) <= 0.5;
        }
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(armPivotMotorSim.getMotorVoltage());
        armSim.update(Constants.LOOP_TIME);

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

    @Override
    public void brakeMode() {
        setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void coastMode() {
        setNeutralMode(NeutralModeValue.Coast);
    }

    private void setNeutralMode(NeutralModeValue value) {
        if (value == neutralMode) return;

        StatusCode code = armPivotMotor.setNeutralMode(value);
        if (code == StatusCode.OK) {
            neutralMode = value;
        } else {
            System.out.println("FAILED to set arm pivot neutral mode: " + code);
        }
    }
}
