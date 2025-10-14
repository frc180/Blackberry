package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
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
    private final TalonFX placerMotor;

    // Simulation variables
    private final double kGearRatio = 1;
    @NotLogged
    private final DCMotor motorModel = DCMotor.getNeo550(1);
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            motorModel, 0.001, kGearRatio
        ),
        motorModel
    );

    public CoralPlacerSubsystem() {
        placerMotor = new TalonFX(Constants.CAN.CORAL_PLACER_ID);

        // Makes acceleration non-instantaneous.
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.OpenLoopRamps = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(Constants.CoralPlacerSubsystem.RAMP_RATE_SECONDS);
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
        TalonFXSimState talonFXSim = placerMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        double motorVoltage = talonFXSim.getMotorVoltage();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage);
        m_motorSimModel.update(Constants.LOOP_TIME);

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    }
}