package frc.robot.subsystems.IntakeAlgaePivot;

import static frc.robot.util.StatusSignals.trackSignal;
import java.util.List;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeAlgaePivotIOTalonFXS implements IntakeAlgaePivotIO {
    
    final double intakeArmGearing = 200;
    final double radToRotations = 1 / (2 * Math.PI);
    final double absoluteEncoderResolution = 2048;

    final double absoluteOffset = 0;

    TalonFX pivotMotorA, pivotMotorB;
    List<TalonFX> pivotMotors;
    MotionMagicVoltage motionMagicControl;
    VoltageOut voltageControl;
    Follower followerControl;
    DutyCycleEncoder absoluteEncoder;

    // Status signals
    StatusSignal<Angle> positionSignal;
    StatusSignal<Voltage> voltageSignal;
    StatusSignal<Double> targetSignal;

    // Simulation-only variables
    TalonFXSimState pivotMotorASim, pivotMotorBSim;
    List<TalonFXSimState> pivotMotorSims;
    SingleJointedArmSim intakeSim;

    public IntakeAlgaePivotIOTalonFXS() {
        pivotMotorA = new TalonFX(Constants.INTAKE_ALGAE_PIVOT_TALON_A, Constants.CANIVORE);
        pivotMotorB = new TalonFX(Constants.INTAKE_ALGAE_PIVOT_TALON_B, Constants.CANIVORE);
        pivotMotors = List.of(pivotMotorA, pivotMotorB);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.Feedback.SensorToMechanismRatio = intakeArmGearing;
        if (Robot.isReal()) {
            configuration.Slot0.kP = 0;
            configuration.Slot0.kI = 0;
            configuration.Slot0.kD = 0;
            configuration.Slot0.kG = 0;
            configuration.Slot0.kV = 0;
            configuration.Slot0.kA = 0;
        } else  {
            configuration.Slot0.kP = 36;
            configuration.Slot0.kI = 0;
            configuration.Slot0.kD = 0;
            configuration.Slot0.kG = 0;
            configuration.Slot0.kV = 0;
            configuration.Slot0.kA = 0;
        }
        configuration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.MotionMagic.MotionMagicCruiseVelocity = 999;
        configuration.MotionMagic.MotionMagicAcceleration = 999;
        configuration.MotionMagic.MotionMagicJerk = 0;
        

        //pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        //pivotMotor.getConfigurator().apply(configuration);
        pivotMotors.forEach(motor -> {
            motor.getConfigurator().apply(configuration);
            motor.setNeutralMode(NeutralModeValue.Coast);
        });

        motionMagicControl = new MotionMagicVoltage(0);
        voltageControl = new VoltageOut(0);
        followerControl = new Follower(Constants.INTAKE_ALGAE_PIVOT_TALON_A, true);
        pivotMotorB.setControl(followerControl);

        absoluteEncoder = new DutyCycleEncoder(Constants.DIO_INTAKE_ALGAE_ENCODER);

        positionSignal = trackSignal(pivotMotorA.getPosition());
        voltageSignal = trackSignal(pivotMotorA.getMotorVoltage());
        targetSignal = trackSignal(pivotMotorA.getClosedLoopReference());

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
            Units.rotationsToRadians(IntakeAlgaePivotSubsystem.stow)
        );
    }


    @Override
    public void update(IntakeAlgaePivotIOInputs inputs){ 
        inputs.position = positionSignal.getValueAsDouble();
        inputs.voltage = voltageSignal.getValueAsDouble();
        inputs.target = targetSignal.getValueAsDouble();
        inputs.absolutePosition = absoluteEncoder.get() - absoluteOffset;
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

    @Override
    public void setSpeed(double speed) {
        pivotMotorA.setControl(voltageControl.withOutput(speed * 12));
    }

    public void runMotorTest() {
        System.out.println("algae intake pivot running");
        pivotMotorA.set(0.25); //pivotMotorB is follower
    }

    @Override
    public void stopMotor() {
        pivotMotorA.stopMotor();
        // pivotMotorB.stopMotor();
    }

    @Override
    public void zero(double offset) {
        pivotMotorA.setPosition(offset, 0);
    }
}
