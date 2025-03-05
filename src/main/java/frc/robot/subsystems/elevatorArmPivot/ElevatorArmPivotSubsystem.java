package frc.robot.subsystems.elevatorArmPivot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotIO.ElevatorArmPivotIOInputs;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class ElevatorArmPivotSubsystem extends SubsystemBase {

    private ElevatorArmPivotIO io;
    private ElevatorArmPivotIOInputs inputs;

    // TODO: read manually from robot
    protected static final Angle FORWARD_LIMIT = Degrees.of(60.7);
    protected static final Angle REVERSE_LIMIT = Degrees.of(-20);
    protected static final Angle HARD_STOP_OFFSET = Degrees.of(60.46875 + 3.955078125 + 2.109375 - 2.63671875);
    private static final double RESYNC_THRESHOLD = Degrees.of(2).in(Rotations);

    public static final double L4_SCORE = Units.degreesToRotations(-16);
    public static final double L3_SCORE = Units.degreesToRotations(-6);
    public static final double L2_SCORE = L3_SCORE;
    public static final double L1_SCORE = Units.degreesToRotations(-1);

    public static final double receiving = Units.degreesToRotations(45 - 3);
    public static final double algaeReceive = Units.degreesToRotations(-70);
    public static final double horizontal = 0;

    public static final double netScore = Units.degreesToRotations(62);
    public static final double netScoreBackwards = Units.degreesToRotations(46.01);

    private static final double IN_POSITION_TOLERANCE = Units.degreesToRotations(2);

    private final Alert notHomedAlert = new Alert("Arm Pivot is not homed!", AlertType.kWarning);

    private Double targetPositionMax = null;
    private Double targetPositionMin = null;

    private double targetPosition = 0;
    private boolean pidMode = false;
    private boolean isHoming = false;
    private boolean homed = false;

    private double absoluteScalar = Robot.isReal() ? (2.29 * .967): 4;
    private double absoluteOffset = Robot.isReal() ? -0.701 : 30 * 4;

    @NotLogged
    private double lastPositionSyncTime = 0;
    private double positionDisagreement = 0;
    private boolean shouldResync = false;
    private double absoluteRatio = 0;
    private double absoluteRatioFiltered = 0;
    private double absoluteRatioSamples = 0;
    private final MedianFilter absoluteRatioFilter = new MedianFilter(15);

    @NotLogged
    public Trigger elevatorArmInPosition = new Trigger(() -> isElevatorArmInPosition());
    @NotLogged
    public Trigger elevatorArmInScoringPosition = new Trigger (() -> isElevatorArmInScoringPosition());
    @NotLogged
    public Trigger atHardstop = new Trigger(() -> inputs.hardStop).debounce(0.2);
    private Trigger atHomingHardstop = new Trigger(this::isAtHomingHardstop).debounce(0.2);

    public ElevatorArmPivotSubsystem() {
        inputs = new ElevatorArmPivotIOInputs();
        if (Robot.isReal()) {
            // io = new ElevatorArmPivotIOSim();
            io = new ElevatorArmPivotIOTalonFX();
        } else {
            io = new ElevatorArmPivotIOTalonFX();
        }
    }

    @NotLogged
    boolean firstPeriodic = true;

    @Override
    public void periodic() {
        io.update(inputs);
        SimVisuals.setElevatorArmPivotDegrees(getDegrees());

        notHomedAlert.set(!homed);

        absoluteRatio = inputs.position / getAbsolutePosition();
        positionDisagreement = getAbsolutePosition() - inputs.position;
        shouldResync = Math.abs(positionDisagreement) > RESYNC_THRESHOLD;

        double currentTime = Timer.getFPGATimestamp();
        if (firstPeriodic) {// || currentTime - lastPositionSyncTime > 0.5) {
            io.zero(getAbsolutePosition());
            homed = true;
            firstPeriodic = false;
            lastPositionSyncTime = currentTime;
        }

        // if (pidMode) {
        //     setArmPositionDirect(targetPosition);
        // }
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command receivePosition() {
        return setPosition(receiving);
    }

    public Command stowPosition() {
        return setPosition(receiving);
    }

    public Command horizontalPosition() {
        return setPosition(horizontal);
    }

    public Command netScorePosition() {
        return setPosition(netScore);
    }

    public Command netScoreBackwardsPosition() {
        return setPosition(netScoreBackwards);
    }

    public Command receiveAlgaePosition() {
        return setPosition(algaeReceive);
    }

    public Command home() {
        return Commands.sequence(
            runOnce(() -> {
                homed = false;
                isHoming = true;
            }),
            runSpeed(0.04).until(atHomingHardstop),
            zero(HARD_STOP_OFFSET).alongWith(Commands.runOnce(() -> {
                homed = true;
                isHoming = false;
            }))
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command calculateAbsoluteRatio() {
        return Commands.sequence(
            runOnce(() -> {
                io.zero(0);
                zeroAbsolute();
                io.setSpeed(0.04);
                absoluteRatioFiltered = 0;
                absoluteRatioSamples = 0;
                absoluteRatioFilter.reset();
            }),
            Commands.waitSeconds(0.1),
            runEnd(
                () -> {
                    absoluteRatioSamples++;
                    absoluteRatioFiltered = absoluteRatioFilter.calculate(inputs.position / getAbsolutePosition());
                },
                () -> io.setSpeed(0)
            ).until(atHardstop)
        ).ignoringDisable(true);
    }

    public Command zero(Angle angle) {
        return zero(angle.in(Rotations));
    }

    public Command zero(double rotations) {
        return runOnce(() -> io.zero(rotations));
    }

    @NotLogged
    private final double MAX_ERROR_THRESHOLD = Inches.of(20).in(Meters);

    public Command matchElevatorPreset() {
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        return run(() -> {
            Distance elevatorTarget = elevator.getTargetPosition();
            // double targetError = elevator.getTargetErrorMeters();

            // Experimental - prevent arm from sticking out until we're past any levels that may
            // contain algae that'd collide with the arm
            // if (targetError > MAX_ERROR_THRESHOLD) {
            //     setArmPositionDirect(receiving);
            //     return;
            // }

            if (elevatorTarget == ElevatorSubsystem.L4) {
                setArmPositionDirect(L4_SCORE);
            } else if (elevatorTarget == ElevatorSubsystem.L3) {
                setArmPositionDirect(L3_SCORE);
            } else if (elevatorTarget == ElevatorSubsystem.L2) {
                setArmPositionDirect(L2_SCORE);
            } else if (elevatorTarget == ElevatorSubsystem.L1) {
                setArmPositionDirect(L1_SCORE);
            } else {
                setArmPositionDirect(receiving);
            }
        });
    }

    public Command setPosition(double position) {
        return run(() -> setArmPositionDirect(position));
    }

    public Command setSpeed(double speed) {
        return run(() -> {
            io.setSpeed(speed);
            pidMode = false;
        });
    }

    public Command runSpeed(double speed) {
        return runEnd(
            () -> {
                io.setSpeed(speed);
                pidMode = false;
            },
            () -> io.stopMotor()
        );
    }

    public Command stop() {
        return run(() -> {
            io.stopMotor();
            pidMode = false;
        });
    }

    public Command brakeMode() {
        return Commands.runOnce(io::brakeMode);
    }

    public Command coastMode() {
        return Commands.runOnce(io::coastMode);
    }

    public void setPositionLimits(Double min, Double max) {
        targetPositionMin = min;
        targetPositionMax = max;
    }

    public void setArmPositionDirect(double position) {
        targetPosition = position;
        pidMode = true;
        if (targetPositionMin != null) position = Math.max(position, targetPositionMin);
        if (targetPositionMax != null) position = Math.min(position, targetPositionMax);
        io.setPosition(position);
    }

    public boolean isElevatorArmInPosition() {
        return Math.abs(targetPosition - inputs.position) <= IN_POSITION_TOLERANCE;
    }

    public boolean isAtReceivingPosition() {
        if (!isElevatorArmInPosition()) return false;
        
        return (targetPosition == receiving || targetPosition == algaeReceive);
    }

    public boolean isElevatorArmInScoringPosition() {
        if (!isElevatorArmInPosition()) return false;

        return targetPosition == L1_SCORE || targetPosition == L2_SCORE || targetPosition == L3_SCORE || 
               targetPosition == L4_SCORE || targetPosition == netScore || targetPosition == netScoreBackwards;
    }

    @NotLogged
    public double getTargetPosition() {
        return targetPosition;
    }

    public double getDegrees() {
        return Units.rotationsToDegrees(inputs.position);
    }

    public double getTargetDegrees() {
        return Units.rotationsToDegrees(targetPosition);
    }

    public double getAbsolutePosition() {
        return (inputs.absolutePosition * absoluteScalar) - absoluteOffset;
    }

    public void zeroAbsolute() {
        absoluteOffset = (inputs.absolutePosition * absoluteScalar);
    }

    public boolean isAtHomingHardstop() {
        // return Math.abs(inputs.velocity) <= 0.004 && isHoming;
        return isHoming && inputs.hardStop;
    }

    @NotLogged
    public boolean isHomed() {
        return homed;
    }
}
