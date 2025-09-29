package frc.robot.subsystems;

import static com.spamrobotics.util.StatusSignals.trackSignal;
import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.spamrobotics.util.simulation.MapleSimSwerveDrivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    public enum HeadingTarget {
        GYRO,
        POSE
    }

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Meters per second desired top speed
    public static final double MAX_SPEED_ACCEL = 5.5;
    public static final double MAX_ANGULAR_RATE = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity (1.5 * Math.PI)
    public static final double MAX_ANGULAR_ACCEL = MAX_ANGULAR_RATE * 8;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.ApplyRobotSpeeds applyClosedLoopSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    private SwerveDriveState cachedState = null;
    private Rotation2d gyroOffset = new Rotation2d();
    private Double targetHeading = null;
    private HeadingTarget targetHeadingType = HeadingTarget.POSE;
    private double headingError = 0;
    private Pose2d mapleSimPose = null;

    @NotLogged
    private final ProfiledPIDController rotationProfiledPid;

    private final StatusSignal<Angle> gyroAngleSignal;
    private final StatusSignal<AngularVelocity> gyroRateSignal;

    // Logging
    private double xPosition = 0;
    private double yPosition = 0;
    private double[] moduleSpeeds = new double[] {0, 0, 0, 0};
    private double[] moduleGoalSpeeds = new double[] {0, 0, 0, 0};
    private double moduleSpeedAvg = 0;

    private boolean pigeonConnected = false;
    private Alert pigeonDisconnectedAlert = new Alert("Pigeon gyro disconnected!", AlertType.kError);

    private RobotConfig config;

    PathConstraints constraints = new PathConstraints(MAX_SPEED * 0.8, MAX_SPEED_ACCEL, MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL); //must be in m/s and rad/s

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public DrivetrainSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            if (RobotContainer.MAPLESIM) {
                startMapleSimThread();
            } else {
                startCTRESimThread();
            }
        }
        configureAutoBuilder();

        gyroAngleSignal = trackSignal(getPigeon2().getYaw());
        gyroRateSignal = trackSignal(getPigeon2().getAngularVelocityZWorld());

        rotationProfiledPid = new ProfiledPIDController(5, 0., 0,
                                        new TrapezoidProfile.Constraints(MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL));
        rotationProfiledPid.enableContinuousInput(-Math.PI, Math.PI);

        if (Robot.isSimulation()) {
            resetPose(new Pose2d(6.77, 4.2, Rotation2d.fromDegrees(90)));
        }
        zeroGyroscope();
    }

    public void drive(ChassisSpeeds speeds) {        
        speeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME);
        setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
    }

    public void driveClosedLoop(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME);
        setControl(applyClosedLoopSpeeds.withSpeeds(speeds));
    }

    @NotLogged
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeDegrees());
    }

    public double getGyroscopeDegrees() {
        return gyroAngleSignal.getValueAsDouble() - gyroOffset.getDegrees();
    }

    @NotLogged
    public double getGyroscopeDegreesWrapped() {
        return MathUtil.inputModulus(getGyroscopeDegrees(), -180, 180);
    }

    public double getGyroscopeRate() {
        return gyroRateSignal.getValueAsDouble();
    }

    public void zeroGyroscope() {
        setGyroscopeAngle(0.0);
    }

    public void zeroGyroscopeUsingPose() {
        double angle = getPose().getRotation().getDegrees() - (Robot.isBlue() ? 0 : 180);
        setGyroscopeAngle(angle);
    }

    public void setGyroscopeAngle(double degrees) {
        double newZero = MathUtil.inputModulus(getPigeon2().getRotation2d().getDegrees() - degrees, -180, 180);
        gyroOffset = Rotation2d.fromDegrees(newZero);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.5); // Wait for simulation to update
        }
        super.resetPose(pose);
    }

    @NotLogged
    public boolean isPigeonConnected() {
        return pigeonConnected;
    }

    @NotLogged
    public Double getTargetHeading() {
        return targetHeading;
    }

    @NotLogged
    public HeadingTarget getTargetHeadingType() {
        return targetHeadingType;
    }
    
    public void setTargetHeading(Double targetHeading) {
        setTargetHeading(targetHeading, HeadingTarget.GYRO);
    }

    public void setTargetHeading(Double targetHeading, HeadingTarget type) {
        Double oldHeading = this.targetHeading;
        HeadingTarget oldType = this.targetHeadingType;

        if (oldType != type) {
            resetHeadingPID(type);
        }

        this.targetHeading = targetHeading == null ? null : MathUtil.inputModulus(targetHeading, -180, 180);
        targetHeadingType = type;
        if (this.targetHeading == null) {
            headingError = 0;
        } else if (oldHeading == null || this.targetHeading == null || this.targetHeading - oldHeading != 0 || oldType != type) {
            headingError = 999; // reset heading error to make sure we don't think we're at the new target immediately
        }
    }

    @NotLogged
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return getFieldRelativeSpeeds(getCachedState());
    }

    public ChassisSpeeds getFieldRelativeSpeeds(SwerveDriveState state) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    public void resetPIDs(HeadingTarget type) {
        resetHeadingPID(type);
    }

    public void resetHeadingPID(HeadingTarget type) {
        resetHeadingPID(type == HeadingTarget.GYRO ? getGyroscopeDegrees() : getPose().getRotation().getDegrees());
    }

    public void resetHeadingPID(Rotation2d rotation) {
        resetHeadingPID(rotation.getDegrees());
    }

    public void resetHeadingPID(double degrees) {
        rotationProfiledPid.reset(Units.degreesToRadians(degrees));
    }

    public double calculateHeadingPID(Rotation2d heading, double targetDegrees) {
        return calculateHeadingPID(heading.getDegrees(), targetDegrees);
    }

    public double calculateHeadingPID(double headingDegrees, double targetDegrees) {
        headingError = MathUtil.inputModulus(targetDegrees - headingDegrees, -180, 180);    
        return rotationProfiledPid.calculate(
            Math.toRadians(headingDegrees), 
            Math.toRadians(targetDegrees)
        );
    }

    public Pose2d getPose() {
        return getCachedState().Pose;
    }

    @NotLogged
    public SwerveDriveState getCachedState() {
        if (cachedState == null) cachedState = getState();
        return cachedState;
    }

    public void clearCache() {
        cachedState = null;
    }

    /**
     * When running in sim, this will return the pose of the robot as simulated by MapleSim (if it is running).
     * Otherwise, if in real life or if not using MapleSim, this returns the same as {@link #getPose()}.
     */
    public Pose2d getSimPose() {
        return mapleSimPose != null ? mapleSimPose : getPose();
    }

    public Trigger withinTargetHeadingTolerance(Angle angle) {
        return withinTargetHeadingTolerance(angle.in(Degrees));
    }

    public Trigger withinTargetHeadingTolerance(double degrees) {
        return new Trigger(() -> {
           if (targetHeading == null) {
               return false;
           }
           return Math.abs(headingError) <= degrees;
        });
    }

    public Trigger aboveSpeed(LinearVelocity velocity) {
        return belowSpeed(velocity).negate();
    }

    public Trigger aboveSpeed(double metersPerSecond) {
        return belowSpeed(metersPerSecond).negate();
    }

    public Trigger belowSpeed(LinearVelocity velocity) {
        return belowSpeed(velocity.in(MetersPerSecond));
    }

    public Trigger belowSpeed(double metersPerSecond) {
        return new Trigger(() -> isBelowSpeed(metersPerSecond));
    }

    public boolean isBelowSpeed(double metersPerSecond) {
        return Math.abs(getVelocity()) < metersPerSecond;
    }

    @NotLogged
    public double getVelocity() {
        ChassisSpeeds speeds = getCachedState().Speeds;
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    private void configureAutoBuilder() {
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getPose(),   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getCachedState().Speeds, // Supplier of current robot speeds
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(2.5, 0, 0), //translation
                    new PIDConstants(4, 0, 0)), //rotation, was 5
                config,
                Robot::isRed, //path flips for red/blue alliance
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command targetHeading(Double heading, HeadingTarget type) {
        return Commands.runOnce(() -> setTargetHeading(heading, type));
    }
    
    public Command targetHeadingContinuous(Double heading, HeadingTarget type) {
        return Commands.run(() -> setTargetHeading(heading, type));
    }
    
    public Command targetHeadingContinuous(Supplier<Double> headingSupplier, HeadingTarget type) {
        return Commands.run(() -> setTargetHeading(headingSupplier.get(), type));
    }

    @Override
    public void periodic() {
        pigeonConnected = gyroAngleSignal.getTimestamp().getLatency() <= 0.5;
        pigeonDisconnectedAlert.set(!pigeonConnected);

        if (RobotState.isDisabled()) {
            zeroGyroscopeUsingPose();
        }

        SwerveDriveState state = getCachedState();
        Pose2d pose = state.Pose;
        xPosition = pose.getX();
        yPosition = pose.getY();

        SwerveModuleState[] moduleStates = state.ModuleStates;
        SwerveModuleState[] moduleTargets = state.ModuleTargets;
        moduleSpeedAvg = 0;
        for (int i = 0; i < moduleStates.length; i++) {
            moduleSpeeds[i] = Math.abs(moduleStates[i].speedMetersPerSecond);
            moduleSpeedAvg += Math.abs(moduleSpeeds[i]);
            moduleGoalSpeeds[i] = Math.abs(moduleTargets[i].speedMetersPerSecond);
        }
        moduleSpeedAvg /= moduleStates.length;

        if (mapleSimSwerveDrivetrain != null) {
            mapleSimPose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }
    }

    // ==================== EVERYTHING BELOW THIS LINE IS SIMULATION-RELATED ====================

    public SwerveDriveSimulation getDriveSim() {
        if (mapleSimSwerveDrivetrain != null) {
            return mapleSimSwerveDrivetrain.mapleSimDrive;
        }
        return null;
    }

    // Original CTRE code
    private void startCTRESimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    @SuppressWarnings("unchecked")
    private void startMapleSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Pounds.of(115), // robot weight
                Inches.of(30), // bumper length
                Inches.of(30), // bumper width
                DCMotor.getKrakenX60Foc(1), // drive motor type
                DCMotor.getKrakenX60Foc(1), // steer motor type
                1.916, // wheel COF, Vex Grip V2
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
        );
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
