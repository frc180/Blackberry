package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import com.spamrobotics.util.Helpers;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.simulation.MapleSimSwerveDrivetrain;

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

    public enum PoseTarget {
        STANDARD,
        REEF
    }

    // STOLEN FROM SONIC, NOT CORRECT
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Meters per second desired top speed
    public static final double MAX_SPEED_ACCEL = 10; // Meters per second squared max acceleration, was 8
    public static final double MAX_ANGULAR_RATE = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity (1.5 * Math.PI)
    public static final double MAX_ANGULAR_ACCEL = MAX_ANGULAR_RATE * 8; // was * 4

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds autoApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();


    private Rotation2d gyroOffset = new Rotation2d();
    private Double targetHeading = null;
    private HeadingTarget targetHeadingType = HeadingTarget.POSE;
    private double headingError = 0;
    private PoseTarget poseTargetType = PoseTarget.STANDARD;
    private Pose2d targetPose = null;

    private Pose2d mapleSimPose = null;

    private ProfiledPIDController xPidController, yPidController, driverRotationPidController;
    private double xPosition = 0;
    private double yPosition = 0;
    private double xPidTarget = 0;
    private double yPidTarget = 0;

    private RobotConfig config;

    PathConstraints constraints = new PathConstraints(MAX_SPEED, MAX_SPEED_ACCEL, MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL); //must be in m/s and rad/s
    
    public PathPlannerPath path;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

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

        double translationP = 3.5; // 3.5 almost stable //3.2 good
        double translationI = 0.0;
        double translationD = 0.05;

        if (Robot.isSimulation()) {
            translationP = 10;
            translationI = 0.0;
            translationD = 1.6;
        }

        xPidController = new ProfiledPIDController(translationP, translationI, translationD,
                                        new TrapezoidProfile.Constraints(MAX_SPEED, MAX_SPEED_ACCEL));
        yPidController = new ProfiledPIDController(translationP, translationI, translationD,
                                        new TrapezoidProfile.Constraints(MAX_SPEED, MAX_SPEED_ACCEL));

        driverRotationPidController = new ProfiledPIDController(5, 0., 0, // was 10 // was 5
                                        new TrapezoidProfile.Constraints(MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL)); // formerly 9999
        driverRotationPidController.enableContinuousInput(-Math.PI, Math.PI);

        if (Robot.isSimulation()) {
            resetPose(new Pose2d(6.77, 4.2, Rotation2d.fromDegrees(90)));
            zeroGyroscope();
        }

    }

    public void drive(ChassisSpeeds speeds) {
        // Helpers.discretizeOverwrite(speeds, Constants.LOOP_TIME);
        ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME);
        setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeDegrees());
    }

    public double getGyroscopeDegrees() {        
        return (-this.getPigeon2().getAngle()) - gyroOffset.getDegrees();
    }

    public void zeroGyroscope() {
        gyroOffset = this.getPigeon2().getRotation2d();
        // seedFieldRelative(); // last year
        // seedFieldCentric(); // this year, acts weird
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.5); // Wait for simulation to update
        }
        super.resetPose(pose);
    }

    public boolean pigeonConnected() {
        return getPigeon2().isConnected();
    }

    public Double getTargetHeading() {
        return targetHeading;
    }

    public HeadingTarget getTargetHeadingType() {
        return targetHeadingType;
    }
    
    public void setTargetHeading(Double targetHeading) {
        setTargetHeading(targetHeading, HeadingTarget.GYRO);
    }

    public void setTargetHeading(Double targetHeading, HeadingTarget type) {
        this.targetHeading = targetHeading == null ? null : MathUtil.inputModulus(targetHeading, -180, 180);
        targetHeadingType = type;
        if (this.targetHeading == null) {
            headingError = 0;
        } else {
            headingError = 999; // reset heading error to make sure we don't think we're at the new target immediately
        }
    }

    public void resetPIDs(HeadingTarget type) {
        SwerveDriveState state = getState();
        // TODO: this speed transform seems to work, but best I can tell it should be using
        // getGyroscopeRotation() instead of the pose rotation...
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        xPidController.reset(state.Pose.getX(), speeds.vxMetersPerSecond);
        yPidController.reset(state.Pose.getY(), speeds.vyMetersPerSecond);
        resetHeadingPID(type);
    }

    public void resetHeadingPID(HeadingTarget type) {
        resetHeadingPID(type == HeadingTarget.GYRO ? getGyroscopeRotation() : getPose().getRotation());
    }

    public void resetHeadingPID(Rotation2d rotation) {
        driverRotationPidController.reset(rotation.getRadians());
    }

    public double calculateHeadingPID(Rotation2d heading, double targetDegrees) {
        return calculateHeadingPID(heading.getDegrees(), targetDegrees);
    }

    public double calculateHeadingPID(double headingDegrees, double targetDegrees) {
        headingError = targetDegrees - headingDegrees;    
        return driverRotationPidController.calculate(
            Math.toRadians(headingDegrees), 
            Math.toRadians(targetDegrees)
        );
    }
    
    public ChassisSpeeds calculateChassisSpeeds(Pose2d currentPose, Pose2d pidPose) {
        double xFeedback = xPidController.calculate(currentPose.getX(), pidPose.getX());
        double yFeedback = yPidController.calculate(currentPose.getY(), pidPose.getY());
        double thetaFeedback = calculateHeadingPID(currentPose.getRotation(), pidPose.getRotation().getDegrees());

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, thetaFeedback, currentPose.getRotation());
    }

    /**
     * Returns if the drivetrain is currently targeting a pose that is for Reef scoring.
     * @return If the drivetrain is targeting a Reef pose. If the pose is not specifically for the reef,
     * or the pose is null, this returns false.
     */
    public boolean isTargetingReefPose() {
        return poseTargetType == PoseTarget.REEF && targetPose != null;
    }

    /**
     * Returns the pose the drivetrain is currently targeting (based on the DriveToPose command)
     * @return The pose the drivetrain is currently targeting. If no command that targets a pose is running,
     * this will return null.
     */
    public Pose2d getTargetPose() {
        return targetPose;
    }

    /**
     * Sets the target pose for the drivetrain. This is for tracking purposes only - setting this alone will
     * not cause the robot to move to the pose. To follow a pose, use the DriveToPose command.
     * @param target
     */
    public void setTargetPose(Pose2d target) {
        targetPose = target;
    }

    /**
     * Returns the type of pose we're targeting.
     * @return The type of pose the drivetrain is targeting. STANDARD by default, or REEF for reef alignment.
     */
    public PoseTarget getPoseTargetType() {
        return poseTargetType;
    }

    public void setPoseTargetType(PoseTarget type) {
        poseTargetType = type;
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * When running in sim, this will return the pose of the robot as simulated by MapleSim (if it is running).
     * Otherwise, if in real life or if not using MapleSim, this returns the same as {@link #getPose()}.
     */
    public Pose2d getSimPose() {
        return mapleSimPose != null ? mapleSimPose : getPose();
    }
    
    public Trigger targetingReef() {
        return new Trigger(this::isTargetingReefPose);
    }

    public Trigger withinTargetPoseTolerance(Double xMeters, Double yMeters, Double degrees) {
        return new Trigger(() -> {
            return Helpers.withinTolerance(getPose(), targetPose, xMeters, yMeters, degrees);
        });
    }

    public Trigger withinTargetPoseDistance(double meters) {
        return new Trigger(() -> {
            if (targetPose == null) {
                return false;
            }
            return getPose().getTranslation().getDistance(targetPose.getTranslation()) <= meters;
        });
    }

    private void configureAutoBuilder() {
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getPose(),   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(2.5, 0, 0), //translation
                    new PIDConstants(5, 0, 0)), //rotation
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, //path flips for red/blue alliance
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

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        if (mapleSimSwerveDrivetrain != null) {
            mapleSimPose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }
        xPosition = getPose().getX();
        yPosition = getPose().getY();
        xPidTarget = xPidController.getSetpoint().position;
        yPidTarget = yPidController.getSetpoint().position;
    }

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
                1.9, // wheel COF
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

    /*public PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, new Rotation2d().fromDegrees(240)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );*/

    public PathPlannerPath getPath(double endVel, Rotation2d endRotation, boolean preventFlipping, List<Waypoint> waypoints) {
        //note that waypoints must contain at least 2 pose2ds wrapped inside PathPlannerPath.waypointsfromPoses(waypoints)
        path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(endVel, endRotation));
        path.preventFlipping = preventFlipping;
        return path;
    }

    public Command followPath(double endVel, Rotation2d endRotation, boolean preventFlipping, List<Waypoint> waypoints) {
        return AutoBuilder.followPath(new PathPlannerPath(waypoints, constraints, null, new GoalEndState(endVel, endRotation)));
    }

    public Command followPath(double endVel, Pose2d endPose, boolean preventFlipping, List<Waypoint> waypoints) {
        Command pathCommand = followPath(endVel, endPose.getRotation(), preventFlipping, waypoints);
        return Commands.parallel(
            Commands.runOnce(() -> {
                Pose2d currentEndPose = endPose;
                if (!preventFlipping && Robot.isRed()) currentEndPose = FlippingUtil.flipFieldPose(endPose);
                setTargetPose(currentEndPose);
            }),
            pathCommand
        );
    }
}
