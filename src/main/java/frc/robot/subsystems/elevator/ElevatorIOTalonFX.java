package frc.robot.subsystems.elevator;

import java.util.List;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorIOTalonFX implements ElevatorIO {

    TalonFX motorA, motorB;
    List<TalonFX> motors;
    PositionVoltage positionControl;
    MotionMagicVoltage motionMagicControl;
    VoltageOut voltageControl;
    Follower followerControl;

    TalonFXSimState motorASim, motorBSim;
    List<TalonFXSimState> motorSims;
    ElevatorSim elevatorSim;

    public ElevatorIOTalonFX() {
        motorA = new TalonFX(Constants.ELEVATOR_TALON_A);
        motorB = new TalonFX(Constants.ELEVATOR_TALON_B);
        motors = List.of(motorA, motorB);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 15;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kG = 1.83;
        slot0Configs.kV = 0.6;
        slot0Configs.kA = 0.24;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 9999;
        motionMagicConfigs.MotionMagicAcceleration = 3.33;
        motionMagicConfigs.MotionMagicJerk = 0;

        motors.forEach(motor -> {
            motor.setNeutralMode(NeutralModeValue.Brake);
            TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(slot0Configs);
            configurator.apply(motionMagicConfigs);
        });

        positionControl = new PositionVoltage(0);
        motionMagicControl = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);
        followerControl = new Follower(Constants.ELEVATOR_TALON_A, true);

        motorB.setControl(followerControl);

        // Everything past this point is just for simulation setup
        if (Robot.isReal()) return;

        motorASim = motorA.getSimState();
        motorBSim = motorB.getSimState();
        motorSims = List.of(motorASim, motorBSim);

        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            12,
            7,
            0.5,
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
    public void setPosition(double encoderPosition) {
        // motorA.setControl(positionControl.withPosition(encoderPosition));
        motorA.setControl(motionMagicControl.withPosition(encoderPosition));
    }

    @Override
    public void update(ElevatorIOInputs inputs) {
        inputs.position = motorA.getPosition(true).getValueAsDouble();
        inputs.voltage = motorA.getMotorVoltage(true).getValueAsDouble();
        inputs.target = motorA.getClosedLoopReference(true).getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motorASim.getMotorVoltage());
        elevatorSim.update(0.020);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps())
        );

        motorSims.forEach(motorSim -> {
            motorSim.setRawRotorPosition(elevatorSim.getPositionMeters());
            // motorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * 300); // Breaks motor voltage
            motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            // motorSim.setSupplyVoltage(12);
        });
    }
}
