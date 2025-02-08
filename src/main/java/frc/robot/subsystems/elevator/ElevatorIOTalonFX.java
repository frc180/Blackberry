package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorIOTalonFX implements ElevatorIO {

    final double elevatorGearing = 5;
    final double metersToMotorRotations = 1 / Meters.of(0.053848).in(Meters);

    TalonFX motorA, motorB;
    List<TalonFX> motors;
    MotionMagicVoltage motionMagicControl;
    MotionMagicExpoVoltage motionMagicExpoControl;
    VoltageOut voltageControl;
    Follower followerControl;

    TalonFXSimState motorASim, motorBSim;
    List<TalonFXSimState> motorSims;
    ElevatorSim elevatorSim;

    public ElevatorIOTalonFX() {
        motorA = new TalonFX(Constants.ELEVATOR_TALON_A);
        motorB = new TalonFX(Constants.ELEVATOR_TALON_B);
        motors = List.of(motorA, motorB);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = elevatorGearing;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.49;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.01;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.Slot0.kP = 0.97;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kG = 0.4031;
        config.Slot0.kV = 0.78;
        config.Slot0.kA = 0.0017552;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.MotionMagic.MotionMagicJerk = 0;
        // normal MotionMagic config
        // config.MotionMagic.MotionMagicCruiseVelocity = 999;
        // config.MotionMagic.MotionMagicAcceleration = 8;
        // MotionMagicExpo config
        config.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        config.MotionMagic.MotionMagicExpo_kV = 2.5;
        config.MotionMagic.MotionMagicExpo_kA = 0.5;

        motors.forEach(motor -> {
            motor.setNeutralMode(NeutralModeValue.Brake);
            motor.getConfigurator().apply(config);
        });

        motionMagicControl = new MotionMagicVoltage(0);
        motionMagicExpoControl = new MotionMagicExpoVoltage(0);
        voltageControl = new VoltageOut(0);
        followerControl = new Follower(Constants.ELEVATOR_TALON_A, false);

        motorB.setControl(followerControl);

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
    }

    @Override
    public void setPower(double power) {
        motorA.setControl(voltageControl.withOutput(power * 12.0));
    }

    @Override
    public void setVoltage(double volts) {
        motorA.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setPosition(double encoderPosition) {
        // motorA.setControl(motionMagicControl.withPosition(encoderPosition));
        motorA.setControl(motionMagicExpoControl.withPosition(encoderPosition));
    }

    @Override
    public void update(ElevatorIOInputs inputs) {
        inputs.position = motorA.getPosition(true).getValueAsDouble();
        inputs.velocity = motorA.getVelocity(true).getValueAsDouble();
        inputs.voltage = motorA.getMotorVoltage(true).getValueAsDouble();
        inputs.target = motorA.getClosedLoopReference(true).getValueAsDouble();
    }

    // Simulation-only code which runs periodically before update() is called
    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motorASim.getMotorVoltage());
        elevatorSim.update(0.020);;

        motorSims.forEach(motorSim -> {
            motorSim.setRawRotorPosition(elevatorSim.getPositionMeters() * metersToMotorRotations);
            motorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * metersToMotorRotations);
            motorSim.setSupplyVoltage(12);
        });
    }

    @Override
    public void runMotorTest() {
        System.out.println("running elevator motors");
        motorA.set(0.25); //motorB is follower
    }

    @Override
    public void stopMotor() {
        motorA.stopMotor();
        motorB.stopMotor();
    }
}
