package frc.robot.subsystems.IntakeAlgaePivot;

import java.util.List;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeAlgaePivotIOTalonFXS implements IntakeAlgaePivotIO{
    
    final double intakeArmGearing = 100;
    final double radToRotations = 1 / (2 * Math.PI);

    TalonFX pivotMotorA, pivotMotorB;
    List<TalonFX> pivotMotors;
    MotionMagicVoltage motionMagicControl;
    VoltageOut voltageControl;
    Follower followerControl;

    // Simulation objects
    TalonFXSimState pivotMotorASim, pivotMotorBSim;
    List<TalonFXSimState> pivotMotorSims;
    SingleJointedArmSim intakeSim;

    public IntakeAlgaePivotIOTalonFXS() {

        pivotMotorA = new TalonFX(Constants.INTAKE_ALGAE_PIVOT_TALON_A);
        pivotMotorB = new TalonFX(Constants.INTAKE_ALGAE_PIVOT_TALON_B);
        pivotMotors = List.of(pivotMotorA, pivotMotorB);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = intakeArmGearing / 360; // Convert to degrees
        configuration.Slot0.kP = 0.1;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0;
        configuration.Slot0.kG = 0;
        configuration.Slot0.kV = 0;
        configuration.Slot0.kA = 0;
        configuration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.MotionMagic.MotionMagicCruiseVelocity = 999;
        configuration.MotionMagic.MotionMagicAcceleration = 999;
        configuration.MotionMagic.MotionMagicJerk = 0;
        

        //pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        //pivotMotor.getConfigurator().apply(configuration);
        pivotMotors.forEach(motor -> {
            motor.setNeutralMode(NeutralModeValue.Brake);
            motor.getConfigurator().apply(configuration);
        });

        motionMagicControl = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);
        followerControl = new Follower(Constants.INTAKE_ALGAE_PIVOT_TALON_A, true);
        pivotMotorB.setControl(followerControl);

        //for simulation
        if (Robot.isReal()) return;

        pivotMotorASim = pivotMotorA.getSimState();
        pivotMotorBSim = pivotMotorB.getSimState();
        pivotMotorSims = List.of(pivotMotorASim, pivotMotorBSim);

        intakeSim = new SingleJointedArmSim(
            DCMotor.getNeo550(2),
            intakeArmGearing,
            1,
            .37,
            0,
            2.0944,
            false,
            Units.degreesToRadians(IntakeAlgaePivotSubsystem.stow)
        );
    }


    @Override
    public void update(IntakeAlgaePivotIOInputs inputs){ 
        inputs.position = pivotMotorA.getPosition(true).getValueAsDouble();
        inputs.voltage = pivotMotorA.getMotorVoltage(true).getValueAsDouble();
        inputs.target = pivotMotorA.getClosedLoopReference(true).getValueAsDouble();
    }

    // Simulation only
    @Override
    public void simulationPeriodic() {
        intakeSim.setInput(pivotMotorASim.getMotorVoltage());
        intakeSim.update(Constants.LOOP_TIME);

        pivotMotorSims.forEach(motorSim -> {
            motorSim.setRawRotorPosition(intakeSim.getAngleRads() * radToRotations * intakeArmGearing); 
            motorSim.setRotorVelocity(intakeSim.getVelocityRadPerSec() * radToRotations * intakeArmGearing);
            motorSim.setSupplyVoltage(12);
        });
    }

    @Override
    public void setPosition(double encoderPosition) {
        pivotMotorA.setControl(motionMagicControl.withPosition(encoderPosition));
    }
}
