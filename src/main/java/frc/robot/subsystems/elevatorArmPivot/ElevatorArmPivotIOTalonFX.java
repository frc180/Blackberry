package frc.robot.subsystems.elevatorArmPivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class ElevatorArmPivotIOTalonFX implements ElevatorArmPivotIO{

    final double elevatorArmPivotGearing = 10; //not the actual number

    TalonFX armPivotMotor;
    MotionMagicVoltage motionMagic;
    VoltageOut voltageControl;
    TalonFXSimState armPivotMotorSim;
    SingleJointedArmSim armSim;

    public ElevatorArmPivotIOTalonFX() {
        armPivotMotor = new TalonFX(Constants.ELEVATOR_ARM_PIVOT_TALON);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = elevatorArmPivotGearing;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kG = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicAcceleration = 0;
        config.MotionMagic.MotionMagicJerk = 0;

        armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        armPivotMotor.getConfigurator().apply(config);

        voltageControl = new VoltageOut(0);

        //for sim
        if (Robot.isReal()) {
            return;
        }

        armPivotMotorSim = armPivotMotor.getSimState();

        armSim = new SingleJointedArmSim( //get this infor from cad
            DCMotor.getNeo550(1),
            elevatorArmPivotGearing,
            0,
            0,
            0,
            0,
            true,
            0,
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
    public void setPosition(double encoderPosition) {
        armPivotMotor.setControl(motionMagic.withPosition(encoderPosition));
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(armPivotMotorSim.getMotorVoltage());
        armSim.update(0.020);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps())
        );

        armPivotMotorSim.setRawRotorPosition(armSim.getAngleRads());
        armPivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    
}
