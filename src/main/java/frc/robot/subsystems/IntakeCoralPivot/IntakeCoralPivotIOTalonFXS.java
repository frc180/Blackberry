package frc.robot.subsystems.IntakeCoralPivot;

import frc.robot.subsystems.IntakeCoral.IntakeCoralIO.IntakeIOInputs;
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


public class IntakeCoralPivotIOTalonFXS implements IntakeCoralPivotIO {

    public IntakeCoralPivotIOTalonFXS() {

    }

    @Override
    public void setIntakePosition(double position){
    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void update(IntakeCoralPivotIOInputs inputs) {
    }
    
}
