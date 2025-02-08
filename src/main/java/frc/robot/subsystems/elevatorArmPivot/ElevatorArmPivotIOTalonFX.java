package frc.robot.subsystems.elevatorArmPivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorArmPivotIOTalonFX implements ElevatorArmPivotIO{

    final double elevatorArmPivotGearing = 80;
    final double radToRotations = 1 / (2 * Math.PI);

    TalonFX armPivotMotor;
    MotionMagicVoltage motionMagic;
    VoltageOut voltageControl;

    TalonFXSimState armPivotMotorSim;
    SingleJointedArmSim armSim;

    public ElevatorArmPivotIOTalonFX() {
        armPivotMotor = new TalonFX(Constants.ELEVATOR_ARM_PIVOT_TALON);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = elevatorArmPivotGearing / 360; // convert rotations to degrees
        config.Slot0.kP = 0.45;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kG = 0.9;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotionMagic.MotionMagicCruiseVelocity = 999;
        config.MotionMagic.MotionMagicAcceleration = 999;
        config.MotionMagic.MotionMagicJerk = 0;

        armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        armPivotMotor.getConfigurator().apply(config);

        motionMagic = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);

        // Everything below this line is for simulation only
        if (Robot.isReal()) return;

        armPivotMotorSim = armPivotMotor.getSimState();

        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            elevatorArmPivotGearing,
            1,
            0.4572,
            -Math.PI,
            Math.PI,
            true,
            0
        );
    }

    @Override
    public void update(ElevatorArmPivotIOInputs inputs) {
        inputs.position = armPivotMotor.getPosition(true).getValueAsDouble();
        inputs.voltage = armPivotMotor.getMotorVoltage(true).getValueAsDouble();
        inputs.target = armPivotMotor.getClosedLoopReference(true).getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(armPivotMotorSim.getMotorVoltage());
        armSim.update(0.020);

        armPivotMotorSim.setRawRotorPosition(armSim.getAngleRads() * radToRotations * elevatorArmPivotGearing);
        armPivotMotorSim.setRotorVelocity(armSim.getVelocityRadPerSec() * radToRotations * elevatorArmPivotGearing);
        armPivotMotorSim.setSupplyVoltage(12);
    }

    @Override
    public void runMotorTest() {
        System.out.println("elevator arm pivot running");
        armPivotMotor.set(0.25);
    }

    @Override
    public void stopMotor() {
        armPivotMotor.stopMotor();
    }

    
    @Override
    public void setPosition(double encoderPosition) {
        armPivotMotor.setControl(motionMagic.withPosition(encoderPosition));
    }
}
