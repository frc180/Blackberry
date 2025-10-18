package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.spamrobotics.util.Helpers;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CoralPlacementCycle;

@Logged
public class CoralPlacerSubsystem extends SubsystemBase {
    private final TalonFXS placerMotor;

    // Simulation variables
    private final double kGearRatio = 1;
    @NotLogged
    private final DCMotor motorModel = Helpers.getMinion(1);
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            motorModel, 0.001, kGearRatio
        ),
        motorModel
    );

    public CoralPlacerSubsystem() {
        placerMotor = new TalonFXS(Constants.CAN.CORAL_PLACER_ID);

        // Makes acceleration non-instantaneous.
        TalonFXSConfiguration configs = new TalonFXSConfiguration();
        configs.OpenLoopRamps = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(Constants.CoralPlacerSubsystem.RAMP_RATE_SECONDS);
        configs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        placerMotor.getConfigurator().apply(configs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PlacerMotor Speed", placerMotor.get());
    }

    public void runPlacer(double speed) {
        placerMotor.set(speed);
    }

    public void stop() {
        placerMotor.stopMotor();
    }

    public Command setSpeed(double speed) {
        return new CoralPlacementCycle(this, speed);
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSSimState talonSim = placerMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        double motorVoltage = talonSim.getMotorVoltage();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage);
        m_motorSimModel.update(Constants.LOOP_TIME);

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
        talonSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    }
}