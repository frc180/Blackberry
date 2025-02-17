package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.spamrobotics.util.PIDTuner;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorIOTalonFX implements ElevatorIO {

    final double elevatorGearing = 5;
    final double metersToMotorRotations = 1 / Meters.of(0.0382524).in(Meters);

    TalonFX motorA, motorB;
    List<TalonFX> motors;
    MotionMagicVoltage motionMagicControl;
    MotionMagicExpoVoltage motionMagicExpoControl;
    VoltageOut voltageControl;
    Follower followerControl;
    DigitalInput bottomLimit;
    PIDTuner pidTuner = null;

    // Status signals
    StatusSignal<Angle> positionSignal;
    StatusSignal<AngularVelocity> velocitySignal;
    StatusSignal<Voltage> voltageSignal;
    StatusSignal<Double> targetSignal;

    // Simulation-only variables
    TalonFXSimState motorASim, motorBSim;
    List<TalonFXSimState> motorSims;
    ElevatorSim elevatorSim;

    public ElevatorIOTalonFX() {
        motorA = new TalonFX(Constants.ELEVATOR_REAR, Constants.CANIVORE);
        motorB = new TalonFX(Constants.ELEVATOR_FRONT,Constants.CANIVORE);
        motors = List.of(motorA, motorB);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = Robot.isReal() ? metersToMotorRotations : elevatorGearing;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorSubsystem.SOFT_UPPER_LIMIT;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.01;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        if (Robot.isReal()) {
            config.Slot0.kP = 20;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0;
            config.Slot0.kV = 3;
            config.Slot0.kA = 0;
            config.MotionMagic.MotionMagicExpo_kV = 4;
            config.MotionMagic.MotionMagicExpo_kA = 1.5;
        } else {
            config.Slot0.kP = 0.97;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;
            config.Slot0.kG = 0.4031;
            config.Slot0.kV = 0.55;
            config.Slot0.kA = 0.0017552;
            config.MotionMagic.MotionMagicExpo_kV = 0.78;
            config.MotionMagic.MotionMagicExpo_kA = 0.5;
        }
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        config.MotionMagic.MotionMagicJerk = 0;
        // normal MotionMagic config
        // config.MotionMagic.MotionMagicCruiseVelocity = 999;
        // config.MotionMagic.MotionMagicAcceleration = 8;
        motors.forEach(motor -> {
            motor.setNeutralMode(NeutralModeValue.Brake);
            motor.getConfigurator().apply(config);
        });

        motionMagicControl = new MotionMagicVoltage(0);
        motionMagicExpoControl = new MotionMagicExpoVoltage(0);
        voltageControl = new VoltageOut(0);
        followerControl = new Follower(Constants.ELEVATOR_REAR, false);

        motorB.setControl(followerControl);

        bottomLimit = new DigitalInput(Constants.DIO_ELEVATOR_BOTTOM_LIMIT);
        
        // pidTuner = new PIDTuner(config.Slot0, motorA).withName("Elevator");
        // pidTuner.initializeValues(config.Slot0);

        positionSignal = motorA.getPosition();
        velocitySignal = motorA.getVelocity();
        voltageSignal = motorA.getMotorVoltage();
        targetSignal = motorA.getClosedLoopReference();

        // Everything past this point is just for simulation setup
        if (Robot.isReal()) return;

        motorASim = motorA.getSimState();
        motorBSim = motorB.getSimState();
        motorSims = List.of(motorASim, motorBSim);

        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            elevatorGearing,
            9,
            0.0356,
            0,
            1.499,
            true,
            0
        );
        elevatorSim.update(0);
    }

    @Override
    public void setPower(double power) {
        setVoltage(power * 12);
    }

    @Override
    public void setVoltage(double volts) {
        motorA.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setPosition(double encoderPosition) {
        motorA.setControl(motionMagicExpoControl.withPosition(encoderPosition));
    }

    @Override
    public void update(ElevatorIOInputs inputs) {
        inputs.signalStatus = BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, targetSignal);
        inputs.position = positionSignal.getValueAsDouble();
        inputs.velocity = velocitySignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();

        if (Robot.isReal()) {
            inputs.bottomLimit = !bottomLimit.get();
        } else {
            inputs.bottomLimit = elevatorSim.getPositionMeters() <= 0.01;
        }

        if (pidTuner != null) pidTuner.periodic();
    }

    // Simulation-only code which runs periodically before update() is called
    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motorASim.getMotorVoltage());
        elevatorSim.update(Constants.LOOP_TIME);

        motorSims.forEach(motorSim -> {
            motorSim.setRawRotorPosition(elevatorSim.getPositionMeters() * metersToMotorRotations);
            motorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * metersToMotorRotations);
            motorSim.setSupplyVoltage(12);
        });
    }

    @Override
    public void stopMotor() {
        motorA.stopMotor();
        // motorB.stopMotor();
    }

    @Override
    public void zero() {
        motorA.setPosition(0);
    }
}
