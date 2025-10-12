package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralPlacerSubsystem extends SubsystemBase {
    private final TalonFX placerMotor;

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
}