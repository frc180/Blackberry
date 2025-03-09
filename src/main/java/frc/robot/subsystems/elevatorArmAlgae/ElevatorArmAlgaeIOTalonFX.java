package frc.robot.subsystems.elevatorArmAlgae;

import static frc.robot.util.StatusSignals.trackSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;
import frc.robot.util.simulation.SimLogic;

public class ElevatorArmAlgaeIOTalonFX implements ElevatorArmAlgaeIO {

    TalonFXS motor;
    CANrange canrange;
    VoltageOut voltageControl;
    StatusSignal<Temperature> temperatureSignal;
    StatusSignal<Distance> distanceSignal;

    // Simulation-only variables
    TalonFXSSimState motorSim;
    CANrangeSimState canrangeSim;
    double algaeDistance = -1;
    
    public ElevatorArmAlgaeIOTalonFX() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor = new TalonFXS(Constants.ELEVATOR_ARM_ALGAE, Constants.CANIVORE);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
        voltageControl = new VoltageOut(0);

        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
        rangeConfig.FovParams.FOVRangeX = 6.75;
        rangeConfig.FovParams.FOVRangeY = 6.75;
        rangeConfig.FovParams.FOVCenterY = 11.8;
        rangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        canrange = new CANrange(Constants.ALGAE_ARM_CANRANGE, Constants.CANIVORE);
        canrange.getConfigurator().apply(rangeConfig);

        distanceSignal = trackSignal(canrange.getDistance());
        temperatureSignal = trackSignal(motor.getDeviceTemp());

        // ParentDevice.optimizeBusUtilizationForAll(10.0, motor, canrange);
        
        if (Robot.isReal()) return;

        motorSim = motor.getSimState();
        canrangeSim = canrange.getSimState();
    }

    @Override
    public void update(ElevatorArmAlgaeInputs inputs) {
        double distance = distanceSignal.getValueAsDouble();
        inputs.distance = distance;
        inputs.hasAlgae = distance < 0.1;
        inputs.temperature = temperatureSignal.getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        setVoltage(speed * 12);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

    private void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void simulationPeriodic() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        ElevatorArmPivotSubsystem elevatorArmPivot = RobotContainer.instance.elevatorArmPivot;
        IntakeAlgaeSubsystem algaeIntake = RobotContainer.instance.intakeAlgae;

        double distance = 0.28;

        // If we're already intaking an algae
        if (SimLogic.armHasAlgae) {
            if (algaeDistance != -1) {
                algaeDistance -= 0.0025 * motorSim.getMotorVoltage();
                algaeDistance = Math.max(algaeDistance, 0);
                distance = algaeDistance;
            }
            if (motorSim.getMotorVoltage() < 0) {
                if (elevatorArmPivot.getTargetPosition() == ElevatorArmPivotSubsystem.netScore) {
                    SimLogic.netAlgae(true);
                } else if (elevatorArmPivot.getTargetPosition() == ElevatorArmPivotSubsystem.netScoreBackwards) {
                    SimLogic.netAlgae(false);
                } else {
                    SimLogic.intakeHasAlgae = true;
                }
                SimLogic.armHasAlgae = false;
            }
        } else {
            algaeDistance = -1;
  
            boolean reefAligned = elevator.isElevatorInReefAlgaePosition() && elevatorArmPivot.isElevatorArmInScoringPosition();
            if (reefAligned) {
                int tag = drivetrain.getTargetPoseTag();
                if (tag != -1 && Field.hasReefAlgae(tag)) {
                    Pose2d pose = drivetrain.getPose();
                    Pose2d algaePose = Field.getReefAlgaePose(tag).toPose2d();
                    double distanceToAlgae = pose.getTranslation().getDistance(algaePose.getTranslation());
                    algaeDistance = Math.min(distanceToAlgae, 1);
                    distance = algaeDistance;

                    if (motorSim.getMotorVoltage() > 0) {
                        Field.removeReefAlgae(tag);
                        SimLogic.armHasAlgae = true;
                    }
                }
            }
            
            boolean fromIntake = algaeIntake.hasAlgae.getAsBoolean() && elevatorArmPivot.isAtReceivingPosition();
            if (fromIntake && motorSim.getMotorVoltage() > 0) {
                SimLogic.armHasAlgae = true;
                SimLogic.intakeHasAlgae = false;
                algaeDistance = 0.1;
            }
        }

        double noise = Math.random() * 0.01 - 0.005;
        canrangeSim.setDistance(distance + noise);
    }
}
