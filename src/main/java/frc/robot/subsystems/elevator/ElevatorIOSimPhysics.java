package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSimPhysics implements ElevatorIO {

    final ElevatorSim elevatorSim;
    final PIDController pid;
    final double kG = 0;

    boolean usingPID = false;
    double voltage = 0;
    double target = 0;

    public ElevatorIOSimPhysics() {
        pid = new PIDController(0.1, 0, 0);
        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            6,
            1,
            1.8288,
            0,
            2,
            false,
            0
        );
    }

    @Override
    public void setPower(double power) {
        voltage = Math.max(-1, Math.min(1, power)) * 12;
        usingPID = false;
    }

    @Override
    public void setPosition(double encoderPosition) {
        target = encoderPosition;
        usingPID = true;
    }

    @Override
    public void update(ElevatorIOInputs inputs) {
        if (usingPID) {
            voltage = pid.calculate(inputs.position, target);
            voltage = Math.max(-1, Math.min(1, voltage));
            voltage = (voltage * 12) + kG;
        }
        inputs.position = elevatorSim.getPositionMeters() * 300;
        inputs.voltage = voltage;
        inputs.target = target;
    }

    @Override
    public void simulationPeriodic() {        
        elevatorSim.setInput(voltage);
        elevatorSim.update(0.020);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps())
        );
    }
}
