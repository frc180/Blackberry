package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.spamrobotics.swerve.SwerveModuleStateRequest;
import com.spamrobotics.util.Helpers;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
        REEF,
        PROCESSOR
    }

    // STOLEN FROM SONIC, NOT CORRECT
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Meters per second desired top speed
    public static final double MAX_SPEED_ACCEL = Robot.isReal() ? 7 : 6; // Meters per second squared max acceleration, was 8 -> 7, should try 6
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
    private final SwerveRequest.ApplyRobotSpeeds applyClosedLoopSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveModuleStateRequest moduleStateRequest = new SwerveModuleStateRequest();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
    @NotLogged
    private boolean driveWithSetpointGenerator = false;

    private Rotation2d gyroOffset = new Rotation2d();
    private Double targetHeading = null;
    private HeadingTarget targetHeadingType = HeadingTarget.POSE;
    private double headingError = 0;
    private PoseTarget poseTargetType = PoseTarget.STANDARD;
    private Pose2d targetPose = null;
    private int targetPoseTag = -1;
    private Pose2d intermediatePose = null;

    private Pose2d mapleSimPose = null;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(2);

    @NotLogged
    private final ProfiledPIDController xProfiledPid, yProfiledPid, rotationProfiledPid;
    private final PIDController xPid, yPid;
    private final SimpleMotorFeedforward xyFeedforward;
    private final TrapezoidProfile.Constraints driveToPoseConstraints;
    private final TrapezoidProfile driveToPoseProfile;
    private double translationKV = 0;
    private State xPidGoalState = new State();
    private State yPidGoalState = new State();
    private double xPosition = 0;
    private double yPosition = 0;
    private double xPidTarget = 0;
    private double yPidTarget = 0;

    private boolean pigeonConnected = false;
    private Alert pigeonDisconnectedAlert = new Alert("Pigeon gyro disconnected!", AlertType.kError);

    private RobotConfig config;

    PathConstraints constraints = new PathConstraints(MAX_SPEED, MAX_SPEED_ACCEL, MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL); //must be in m/s and rad/s
    
    public PathPlannerPath path;


    // Wheel radius characterization
    @NotLogged
    private double lastGyroYawRads = 0;
    @NotLogged
    private double accumGyroYawRads = 0;
    @NotLogged
    private double averageWheelPosition = 0;
    @NotLogged
    private double[] startWheelPositions = new double[4];
    private double currentEffectiveWheelRadius = 0;

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

        double translationMaxSpeed = MAX_SPEED;
        double translationP = 4;
        double translationI = 0.0;
        double translationD = 0.05;
        double translationKV = 0;

        if (Robot.isSimulation()) {
            translationMaxSpeed = MAX_SPEED * 0.8;
            translationP = 8;
            translationI = 0.0;
            translationD = 1;
            translationKV = 0.8;
        }

        xyFeedforward = new SimpleMotorFeedforward(0, translationKV, 0);

        driveToPoseConstraints = new TrapezoidProfile.Constraints(translationMaxSpeed, MAX_SPEED_ACCEL);
        driveToPoseProfile = new TrapezoidProfile(driveToPoseConstraints);
        xPid = new PIDController(translationP, translationI, translationD);
        yPid = new PIDController(translationP, translationI, translationD);

        xProfiledPid = new ProfiledPIDController(translationP, translationI, translationD, driveToPoseConstraints);
        yProfiledPid = new ProfiledPIDController(translationP, translationI, translationD, driveToPoseConstraints);
        

        rotationProfiledPid = new ProfiledPIDController(5, 0., 0,
                                        new TrapezoidProfile.Constraints(MAX_ANGULAR_RATE, MAX_ANGULAR_ACCEL));
        rotationProfiledPid.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putData("Drivetrain X Profiled PID", xProfiledPid);
        SmartDashboard.putData("Drivetrain Y Profiled PID", yProfiledPid);
        SmartDashboard.putData("Drivetrain X PID", xPid);
        SmartDashboard.putData("Drivetrain Y PID", yPid);
        SmartDashboard.putNumber("Drivetrain XY Feedforward", translationKV);

        setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            MAX_ANGULAR_RATE // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        // Initialize the previous setpoint to the robot's current speeds & module states
        SwerveDriveState state = getState();
        previousSetpoint = new SwerveSetpoint(state.Speeds, state.ModuleStates, DriveFeedforwards.zeros(config.numModules));

        if (Robot.isSimulation()) {
            resetPose(new Pose2d(6.77, 4.2, Rotation2d.fromDegrees(90)));
            zeroGyroscope();
        }
    }

    public void drive(ChassisSpeeds speeds) {
        if (driveWithSetpointGenerator) {
            driveWithSetpoints(speeds);
            return;
        }
        
        // Helpers.discretizeOverwrite(speeds, Constants.LOOP_TIME);
        speeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME);
        setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
    }

    public void driveClosedLoop(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME);
        setControl(applyClosedLoopSpeeds.withSpeeds(speeds));
    }

    // Allegedly results in smoother driving that causes less wheel slip due to never commanding
    // physically impossible actions. However, so far in testing, seems to just make robot control poorly.
    public void driveWithSetpoints(ChassisSpeeds speeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, Constants.LOOP_TIME);
        setControl(moduleStateRequest.withModuleStates(previousSetpoint.moduleStates()));
    }

    @NotLogged
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeDegrees());
    }

    @NotLogged
    public double getGyroscopeDegrees() {        
        return (-this.getPigeon2().getAngle()) - gyroOffset.getDegrees();
    }

    @NotLogged
    public double getGyroscopeDegreesWrapped() {
        return MathUtil.inputModulus(getGyroscopeDegrees(), -180, 180);
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
        poseBuffer.clear();
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
        this.targetHeading = targetHeading == null ? null : MathUtil.inputModulus(targetHeading, -180, 180);
        targetHeadingType = type;
        if (this.targetHeading == null) {
            headingError = 0;
        } else if (oldHeading == null || this.targetHeading == null || this.targetHeading - oldHeading != 0) {
            headingError = 999; // reset heading error to make sure we don't think we're at the new target immediately
        }
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return getFieldRelativeSpeeds(getState());
    }

    public ChassisSpeeds getFieldRelativeSpeeds(SwerveDriveState state) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    public void resetPIDs(HeadingTarget type) {
        SwerveDriveState state = getState();
        ChassisSpeeds speeds = getFieldRelativeSpeeds(state);
        xProfiledPid.reset(state.Pose.getX(), speeds.vxMetersPerSecond);
        yProfiledPid.reset(state.Pose.getY(), speeds.vyMetersPerSecond);
        resetHeadingPID(type);

        // Reset variables for experimental pose following too
        driveToPoseStart = null;
        xPid.reset();
        yPid.reset();
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

    @Deprecated
    public ChassisSpeeds calculateChassisSpeeds(Pose2d currentPose, Pose2d pidPose) {
        return calculateChassisSpeeds(currentPose, pidPose, 0, 0);
    }
    
    @Deprecated
    public ChassisSpeeds calculateChassisSpeeds(Pose2d currentPose, Pose2d pidPose, double xGoalVelocity, double yGoalVelocity) {
        intermediatePose = pidPose;
        xPidGoalState.position = pidPose.getX();
        xPidGoalState.velocity = xGoalVelocity;
        yPidGoalState.position = pidPose.getY();
        yPidGoalState.velocity = yGoalVelocity;
        double xFeedback = xProfiledPid.calculate(currentPose.getX(), xPidGoalState);
        double yFeedback = yProfiledPid.calculate(currentPose.getY(), yPidGoalState);
        double thetaFeedback = calculateHeadingPID(currentPose.getRotation(), pidPose.getRotation().getDegrees());

        xFeedback += xyFeedforward.calculate(xProfiledPid.getSetpoint().velocity);
        yFeedback += xyFeedforward.calculate(yProfiledPid.getSetpoint().velocity);

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, thetaFeedback, currentPose.getRotation());
    }

    final TrapezoidProfile.State driveToPoseStartState = new State(0, 0);
    final TrapezoidProfile.State driveToPoseGoalState = new State(0, 0);
    final Timer driveToPoseTimer = new Timer();
    Pose2d driveToPoseStart = null;

    /**
     * Experimental method for driving to a pose using a trapezoidal profile, treating the drivetrain as a one-dimensional
     * mechanism instead of applying the profile to the X and Y axes separately.
     * Derived from this code from team 6995: https://github.com/frc6995/Robot-2025/blob/a9871f804924f71e47eb576578ac69bbe7b9249f/src/main/java/frc/robot/subsystems/DriveBaseS.java#L444
     */
    public ChassisSpeeds experimentalCalculateSpeeds(Pose2d currentPose, Pose2d endPose) {
        intermediatePose = endPose;
        
        boolean initializing = driveToPoseStart == null;
        if (initializing) {
            driveToPoseStart = currentPose;
            driveToPoseTimer.restart();
        }

        Translation2d normDirStartToEnd = endPose.getTranslation().minus(driveToPoseStart.getTranslation());
        double distance = normDirStartToEnd.getNorm();
        normDirStartToEnd = normDirStartToEnd.div(distance + 0.001);

        if (initializing) {        
            driveToPoseStartState.position = distance;
            driveToPoseStartState.velocity = MathUtil.clamp(
                Helpers.velocityTowards(driveToPoseStart, getFieldRelativeSpeeds(), endPose.getTranslation()),
                -driveToPoseConstraints.maxVelocity, 
                0
            );
        }

        // Calculate the setpoint (i.e. current distance along the path) we should be targeting
        State setpoint = driveToPoseProfile.calculate(driveToPoseTimer.get(), driveToPoseStartState, driveToPoseGoalState);
        Translation2d setpointTarget = endPose.getTranslation().interpolate(driveToPoseStart.getTranslation(), setpoint.position / distance);

        // Use normal PIDs to calculate the feedback for the X and Y axes to reach the setpoint position
        double xOutput = xPid.calculate(currentPose.getX(), setpointTarget.getX());
        double yOutput = yPid.calculate(currentPose.getY(), setpointTarget.getY());

        // Use feedforward for the X and Y axes to better reach the setpoint speed
        double xTargetVelocity = normDirStartToEnd.getX() * -setpoint.velocity;
        double yTargetVelocity = normDirStartToEnd.getY() * -setpoint.velocity;
        xOutput += xyFeedforward.calculate(xTargetVelocity);
        yOutput += xyFeedforward.calculate(yTargetVelocity);

        // Make sure final X and Y outputs are within the max velocity constraints
        xOutput = MathUtil.clamp(xOutput, -driveToPoseConstraints.maxVelocity, driveToPoseConstraints.maxVelocity);
        yOutput = MathUtil.clamp(yOutput, -driveToPoseConstraints.maxVelocity, driveToPoseConstraints.maxVelocity);

        double thetaOutput = calculateHeadingPID(currentPose.getRotation(), endPose.getRotation().getDegrees());

        return ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, thetaOutput, currentPose.getRotation());
    }

    /**
     * Returns if the drivetrain is currently targeting a pose that is for Reef scoring.
     * @return If the drivetrain is targeting a Reef pose. If the pose is not specifically for the reef,
     * or the pose is null, this returns false.
     */
    public boolean isTargetingReefPose() {
        return poseTargetType == PoseTarget.REEF && targetPose != null;
    }

    public boolean isTargetingProcessorPose() {
        return poseTargetType == PoseTarget.PROCESSOR && targetPose != null;
    }

    @NotLogged
    public int getTargetPoseTag() {
        return targetPoseTag;
    }

    public void setTargetPoseTag(int tag) {
        targetPoseTag = tag;
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
    
    @NotLogged
    public Trigger targetingReef() {
        return new Trigger(this::isTargetingReefPose);
    }

    @NotLogged
    public Trigger targetingProcessor() {
        return new Trigger(this::isTargetingProcessorPose);
    }

    public Trigger withinTargetPoseTolerance(Distance xDistance, Distance yDistance, Angle angle) {
        return new Trigger(() -> {
            return Helpers.withinTolerance(getPose(), targetPose, xDistance, yDistance, angle);
        });
    }

    public Trigger withinTargetPoseTolerance(Double xMeters, Double yMeters, Double degrees) {
        return new Trigger(() -> {
            return Helpers.withinTolerance(getPose(), targetPose, xMeters, yMeters, degrees);
        });
    }

    public Trigger withinTargetPoseDistance(Distance distance) {
        return withinTargetPoseDistance(distance.in(Meters));
    }

    public Trigger withinTargetPoseDistance(double meters) {
        return new Trigger(() -> {
            if (targetPose == null) {
                return false;
            }
            return getPose().getTranslation().getDistance(targetPose.getTranslation()) <= meters;
        });
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

    // Adapted from https://github.com/WaltonRobotics/Reefscape/blob/bca8e2b01ee13f46fc53a4748f4cf2c9eb5de017/src/main/java/frc/robot/subsystems/Swerve.java#L308
    public Command wheelRadiusCharacterization(double omegaDirection) {
        final DoubleSupplier m_gyroYawRadsSupplier = () -> Units.degreesToRadians(getPigeon2().getYaw().getValueAsDouble());
        final SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(0.5);
        final SwerveRequest.RobotCentric m_characterizationReq = new SwerveRequest.RobotCentric()
		    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final double m_characterizationSpeed = 1.5;

		var initialize = runOnce(() -> {
			lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
			accumGyroYawRads = 0;
			currentEffectiveWheelRadius = 0;
            averageWheelPosition = 0;
			for (int i = 0; i < getModules().length; i++) {
				var pos = getModules()[i].getPosition(true);
				startWheelPositions[i] = pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
			}
			m_omegaLimiter.reset(0);
		});

		var executeEnd = runEnd(
			() -> {
				setControl(m_characterizationReq
					.withRotationalRate(m_omegaLimiter.calculate(m_characterizationSpeed * omegaDirection)));
				accumGyroYawRads += MathUtil.angleModulus(m_gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
				lastGyroYawRads = m_gyroYawRadsSupplier.getAsDouble();
				averageWheelPosition = 0;
				double[] wheelPositions = new double[4];
				for (int i = 0; i < getModules().length; i++) {
					var pos = getModules()[i].getPosition(true);
					wheelPositions[i] = pos.distanceMeters * TunerConstants.kDriveRotationsPerMeter;
					averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
				}
				averageWheelPosition = averageWheelPosition / 4.0;
				currentEffectiveWheelRadius = (accumGyroYawRads * TunerConstants.kDriveRadius) / averageWheelPosition;
			}, () -> {
				setControl(m_characterizationReq.withRotationalRate(0));
				if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
					System.out.println("not enough data for characterization " + accumGyroYawRads
                    + "\navgWheelPos: " + averageWheelPosition + "radians");
				} else {
					System.out.println(
						"effective wheel radius: "
							+ currentEffectiveWheelRadius
							+ " inches" + 
                            "\naccumGryoYawRads: " + accumGyroYawRads + "radians" 
                            + "\navgWheelPos: " + averageWheelPosition + "radians");
				}
			});

		return Commands.sequence(
			initialize, executeEnd);
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

        pigeonConnected = getPigeon2().isConnected();
        pigeonDisconnectedAlert.set(!pigeonConnected);

        double kV = SmartDashboard.getNumber("Drivetrain XY Feedforward", 0);
        xyFeedforward.setKv(kV);

        xPosition = getPose().getX();
        yPosition = getPose().getY();
        // State xSetpoint = xProfiledPid.getSetpoint();
        // State ySetpoint = yProfiledPid.getSetpoint();
        // xPidTarget = xSetpoint.position;
        // yPidTarget = ySetpoint.position;
        xPidTarget = xPid.getSetpoint();
        yPidTarget = yPid.getSetpoint();

        poseBuffer.addSample(Timer.getFPGATimestamp(), getPose());

        if (mapleSimSwerveDrivetrain != null) {
            mapleSimPose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }
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

    public PathPlannerPath getPath(double endVel, Rotation2d endRotation, boolean preventFlipping, List<Waypoint> waypoints) {
        //note that waypoints must contain at least 2 pose2ds wrapped inside PathPlannerPath.waypointsfromPoses(waypoints)
        path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(endVel, endRotation));
        path.preventFlipping = preventFlipping;
        return path;
    }

    /**
     * Follows a path defined by a list of waypoints.
     */
    public Command followPath(List<Waypoint> waypoints, double endVel, Pose2d endPose, boolean preventFlipping) {
        PathPlannerPath path = getPath(endVel, endPose.getRotation(), preventFlipping, waypoints);
        Command pathCommand = AutoBuilder.followPath(path);

        return Commands.parallel(
            Commands.runOnce(() -> {
                Pose2d currentEndPose = endPose;
                if (!preventFlipping && Robot.isRed()) currentEndPose = FlippingUtil.flipFieldPose(endPose);
                setTargetPose(currentEndPose);
            }),
            pathCommand
        );
    }

    /**
     * Follows a path defined by a list of poses, with the last pose being the end pose (position & rotation)
     */
    public Command followPath(List<Pose2d> path, double endVel, boolean preventFlipping) {
        int lastIndex = path.size() - 1;
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(path.subList(0, lastIndex));
        return followPath(waypoints, endVel, path.get(lastIndex), preventFlipping);
    }

    public Pose2d getBufferPose(double timestamp) {
        Optional<Pose2d> pose = poseBuffer.getSample(timestamp);
        if (pose.isPresent()) {
            return pose.get();
        }
        return null;
    }
}
