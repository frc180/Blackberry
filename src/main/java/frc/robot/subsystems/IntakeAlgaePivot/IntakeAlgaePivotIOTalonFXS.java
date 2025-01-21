package frc.robot.subsystems.IntakeAlgaePivot;

import java.util.List;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeAlgaePivotIOTalonFXS implements IntakeAlgaePivotIO{
    
    final double intakeArmGearing = 100;

    TalonFX pivotMotorA, pivotMotorB;
    List<TalonFX> pivotMotors;
    MotionMagicVoltage motionMagicControl;
    VoltageOut voltageControl;
    TalonFXSimState pivotMotorASim, pivotMotorBSim;
    List<TalonFXSimState> pivotMotorSims;
    SingleJointedArmSim intakeSim;
    Follower follower;

    public IntakeAlgaePivotIOTalonFXS() {

        pivotMotorA = new TalonFX(Constants.INTAKE_ALGAE_PIVOT_TALON_A);
        pivotMotorB = new TalonFX(Constants.INTAKE_ALGAE_PIVOT_TALON_B);
        pivotMotors = List.of(pivotMotorA, pivotMotorB);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = intakeArmGearing;
        configuration.Slot0.kP = 0;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0;
        configuration.Slot0.kG = 0;
        configuration.Slot0.kV = 0;
        configuration.Slot0.kA = 0;
        configuration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configuration.MotionMagic.MotionMagicCruiseVelocity = 0;
        configuration.MotionMagic.MotionMagicAcceleration = 0;
        configuration.MotionMagic.MotionMagicJerk = 0;
        

        //pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        //pivotMotor.getConfigurator().apply(configuration);
        pivotMotors.forEach(motor -> {
            motor.setNeutralMode(NeutralModeValue.Brake);
            motor.getConfigurator().apply(configuration);
        });

        motionMagicControl = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);
        follower = new Follower(Constants.INTAKE_ALGAE_PIVOT_TALON_A, true);
        pivotMotorB.setControl(follower);

        //for simulation
        if (Robot.isReal()) return;

        pivotMotorASim = pivotMotorA.getSimState();
        pivotMotorBSim = pivotMotorB.getSimState();

        intakeSim = new SingleJointedArmSim(
            DCMotor.getNeo550(2),
            intakeArmGearing,
            0,
            .37,
            0,
            2.0944,
            true,
            2.0944,
            0
        );
    }


    @Override
    public void update(IntakeAlgaePivotIOInputs inputs){ 

        inputs.position = pivotMotorA.getPosition(true).getValueAsDouble();
        inputs.voltage = pivotMotorA.getMotorVoltage(true).getValueAsDouble();
        inputs.target = pivotMotorA.getClosedLoopReference(true).getValueAsDouble();
    }
    
    @Override
    public void setPosition(double encoderPosition) {

        pivotMotorA.setControl(motionMagicControl.withPosition(encoderPosition));
    }

    //simulation only
    @Override
    public void simulationPeriodic() {

        intakeSim.setInput(pivotMotorASim.getMotorVoltage());
        intakeSim.update(0.020);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps())
        );

        //pivotMotorASim.setRawRotorPosition(intakeSim.getAngleRads() * (180/Math.PI)); //convert radians to degrees
        //pivotMotorASim.setSupplyVoltage(RobotController.getBatteryVoltage());

        pivotMotorSims.forEach(motorSim -> {
            motorSim.setRawRotorPosition(intakeSim.getAngleRads()); //convert radians to degrees?
            motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        });
    }
}
