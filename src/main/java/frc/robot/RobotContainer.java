/* 
           _   _ ______ __  __  ____  _   _ ______ 
     /\   | \ | |  ____|  \/  |/ __ \| \ | |  ____|
    /  \  |  \| | |__  | \  / | |  | |  \| | |__   
   / /\ \ | . ` |  __| | |\/| | |  | | . ` |  __|  
  / ____ \| |\  | |____| |  | | |__| | |\  | |____ 
 /_/    \_\_| \_|______|_|  |_|\____/|_| \_|______|

 ====================================================

 2025 FRC Team 180 - S.P.A.M.

 Contributors:
    - Isabella B.
    - Andrew C.
    - Ryan S.
    - Placeholder
    - Placeholder
    - Placeholder
*/
package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.pathplanner.lib.util.FlippingUtil;
import com.spamrobotics.util.Helpers;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Auto.AutoState;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveToCoralPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotSubsystem;
import frc.robot.subsystems.IntakeCoral.IntakeCoralSubsystem;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coralIndexer.CoralIndexerSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CoralScoringPosition;
import frc.robot.util.simulation.SimLogic;
import frc.robot.subsystems.elevatorArm.ElevatorArmSubsystem;
import frc.robot.subsystems.elevatorArmAlgae.ElevatorArmAlgaeSubsystem;
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;

@Logged
public class RobotContainer {
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /**
     * Set this to false to disable MapleSim in simulation
     */
    private static final boolean USE_MAPLESIM = true;
    
    public static final boolean POSING_MODE = false;

    public static final boolean MAPLESIM = USE_MAPLESIM && Robot.isSimulation();
    public static final double DEADBAND = 0.025;

    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandGenericHID testController = new CommandGenericHID(1);
    private final JoystickInputs inputs = new JoystickInputs();

    private final Telemetry logger = new Telemetry(DrivetrainSubsystem.MAX_SPEED);

    @Logged(name = "Drivetrain")
    public final DrivetrainSubsystem drivetrain;
    @Logged(name = "Vision")
    public final VisionSubsystem vision;
    @Logged(name = "Algae Intake")
    public final IntakeAlgaeSubsystem intakeAlgae;
    @Logged(name = "Algae Intake Pivot")
    public final IntakeAlgaePivotSubsystem intakeAlgaePivot;
    @Logged(name = "Coral Intake")
    public final IntakeCoralSubsystem intakeCoral;
    @Logged(name = "Coral Intake Pivot")
    public final IntakeCoralPivotSubsystem intakeCoralPivot;
    @Logged(name = "Elevator")
    public final ElevatorSubsystem elevator;
    @Logged(name = "Elevator Arm Pivot")
    public final ElevatorArmPivotSubsystem elevatorArmPivot;
    @Logged(name = "Elevator Arm")
    public final ElevatorArmSubsystem elevatorArm;
    @Logged(name = "Elevator Arm Algae")
    public final ElevatorArmAlgaeSubsystem elevatorArmAlgae;
    @Logged(name = "Coral Indexer")
    public final CoralIndexerSubsystem coralIndexer;
    @Logged(name = "Climber")
    public final Climber climber;
    public final LEDSubsystem leds;

    @NotLogged
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    @NotLogged
    private final SendableChooser<Boolean> pushChooser = new SendableChooser<Boolean>();

    public final Command autoDoNothing;

    public boolean climbDeployedBool = false;

    @NotLogged
    private final Trigger falseTrigger = new Trigger(() -> false);
    public Trigger robotHasCoral = falseTrigger;
    public Trigger robotHasAlgae = falseTrigger;
    @NotLogged
    public Trigger coralIntakeTrigger = falseTrigger;
    @NotLogged
    public Trigger coralIntakeReady = falseTrigger;
    public Trigger driverRightReef = falseTrigger;
    public Trigger driverLeftReef = falseTrigger;
    @NotLogged
    public Trigger driverAlgaeDescore = falseTrigger;
    @Logged(name = "Reef - Near")
    public Trigger nearReef = null;
    @Logged(name = "Reef - At")
    public Trigger atReef = null;
    public Trigger strugglingNearReef = falseTrigger;
    public final Trigger climbDeployed = new Trigger(() -> climbDeployedBool);

    // Debug
    public Trigger atReefXY;
    public Trigger atReefAngle;
    public Trigger posingAtReef = falseTrigger;

    public static RobotContainer instance;

    public RobotContainer() {
        instance = this;

        drivetrain = TunerConstants.createDrivetrain();

        vision = new VisionSubsystem();
        intakeAlgae = new IntakeAlgaeSubsystem();
        intakeAlgaePivot = new IntakeAlgaePivotSubsystem();
        intakeCoral = new IntakeCoralSubsystem();
        intakeCoralPivot = new IntakeCoralPivotSubsystem();
        elevator = new ElevatorSubsystem();
        elevatorArmPivot = new ElevatorArmPivotSubsystem();
        elevatorArm = new ElevatorArmSubsystem();
        elevatorArmAlgae = new ElevatorArmAlgaeSubsystem();
        coralIndexer = new CoralIndexerSubsystem();
        climber = new Climber();
        leds = new LEDSubsystem();

        autoDoNothing = Commands.none().withName("Do Nothing");

        Pose2d leftBargeSimStart = new Pose2d(7, 5.5, Rotation2d.fromDegrees(180 + 60));
        Pose2d rightBargeSimStart = new Pose2d(7, FlippingUtil.fieldSizeY - 6.5, Rotation2d.fromDegrees(120));

        Pose2d middleBargeSimStart = new Pose2d(7, FlippingUtil.fieldSizeY / 2, Rotation2d.fromDegrees(180));

        Command leftBargeLeft = Auto.bargeCoralAuto(
            Auto.leftBarge5(true),
            true,
            leftBargeSimStart
        );
        Command leftBargeRight = Auto.bargeCoralAuto(
            Auto.leftBarge5(false),
            true,
            leftBargeSimStart
        );
        Command leftBargeLeftNoFront = Auto.bargeCoralAuto(
            Auto.leftBargeAvoidFront(true),
            true,
            leftBargeSimStart
        );
        Command leftBargeRightNoFront = Auto.bargeCoralAuto(
            Auto.leftBargeAvoidFront(false),
            true,
            leftBargeSimStart
        );
        Command rightBargeLeft = Auto.bargeCoralAuto(Auto.rightBarge5(true), false, rightBargeSimStart);
        Command rightBargeRight = Auto.bargeCoralAuto(Auto.rightBarge5(false), false, rightBargeSimStart);

        Command rightBargeLeftNoFront = Auto.bargeCoralAuto(Auto.rightBargeAvoidFront(true), false, rightBargeSimStart);
        Command rightBargeRightNoFront = Auto.bargeCoralAuto(Auto.rightBargeAvoidFront(false), false, rightBargeSimStart);

        Command middleBargeLeft = Auto.bargeCoralAuto(Auto.middleBarge(true), true, middleBargeSimStart);
        Command middleBargeRight = Auto.bargeCoralAuto(Auto.middleBarge(false), true, middleBargeSimStart);

        autoChooser.setDefaultOption("Do Nothing", autoDoNothing);
        autoChooser.addOption("Left Barge - 1st Left", leftBargeLeft);
        autoChooser.addOption("Left Barge - 1st Right", leftBargeRight);
        autoChooser.addOption("Left Barge No Front - 1st Left", leftBargeLeftNoFront);
        autoChooser.addOption("Left Barge No Front - 1st Right", leftBargeRightNoFront);
        autoChooser.addOption("Right Barge - 1st Left", rightBargeLeft);
        autoChooser.addOption("Right Barge - 1st Right", rightBargeRight);
        autoChooser.addOption("Right Barge No Front - 1st Left", rightBargeLeftNoFront);
        autoChooser.addOption("Right Barge No Front - 1st Right", rightBargeRightNoFront);

        autoChooser.addOption("Middle Barge - 1st Left", middleBargeLeft);
        autoChooser.addOption("Middle Barge - 1st Right", middleBargeRight);

        autoChooser.addOption("Drive straight", drivetrain.run(() -> {
            drivetrain.drive(new ChassisSpeeds(0, 1, 0));
        }));

        SmartDashboard.putData("Auto Mode", autoChooser);

        pushChooser.setDefaultOption("No SPAM Ram", false);
        pushChooser.addOption("Yes, SPAM CAN RAM", true);    
        SmartDashboard.putData("SPAM Ram?", pushChooser);

        double offset = Inches.of(203).in(Meters);

        // autoChooser.addOption("Drive 203 inches", Commands.sequence(
        //     Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(2,7, Rotation2d.kZero))),
        //     new DriveToPose(drivetrain, () -> new Pose2d(2 + offset, 7, Rotation2d.kZero)).until(drivetrain.withinTargetPoseDistance(0.02)),
        //     new DriveToPose(drivetrain, () -> new Pose2d(2, 7, Rotation2d.kZero))
        // ));

        if (Robot.isSimulation()) {
            autoChooser.addOption("Drive 6 meters", Commands.sequence(
                Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(2,7, Rotation2d.kZero))),
                new DriveToPose(drivetrain, () -> new Pose2d(8, 6, Rotation2d.kZero)).until(drivetrain.withinTargetPoseDistance(0.02)),
                new DriveToPose(drivetrain, () -> new Pose2d(7, 5, Rotation2d.kCW_90deg)).until(drivetrain.withinTargetPoseDistance(0.02)),
                new DriveToPose(drivetrain, () -> new Pose2d(2, 7, Rotation2d.kZero))
            ));

            autoChooser.addOption("Vision Zeroing Test", Commands.runOnce(() -> {
                Pose2d start = new Pose2d(7, 5.5, Rotation2d.fromDegrees(180 + 60));
                if (Robot.isRed()) start = FlippingUtil.flipFieldPose(start);
                drivetrain.resetPose(start);
            }));
        }

        SmartDashboard.putData("Climber Coast", intakeAlgaePivot.coastMode());
        SmartDashboard.putData("Climber Brake", intakeAlgaePivot.brakeMode());

        configureBindings();

        // drivetrain.singCommand("enemy").schedule();
        if (POSING_MODE) {
            SmartDashboard.putNumber("Posing/Level", 4);
        }
    }

    private void configureBindings() {
        // Robot modes
        final Trigger disabled = RobotModeTriggers.disabled();
        final Trigger teleop = RobotModeTriggers.teleop();
        final Trigger autonomous = RobotModeTriggers.autonomous();
        final Trigger testMode = RobotModeTriggers.test();
        final Trigger algaeMode = driverController.povDown();
        final Trigger coralMode = algaeMode.negate();


        // Driver buttons
        //coral
        final Trigger driverSpit = driverController.y();
        final Trigger driverIntake = driverController.leftTrigger().and(coralMode);
        final Trigger driverIntakeHP = driverController.povRight().and(coralMode); //trigger for intaking from the human player station
        final Trigger driverL1 = driverController.y().and(coralMode);
        final Trigger driverL2 = driverController.leftBumper().and(coralMode);
        final Trigger driverL3 = driverController.rightTrigger().and(coralMode);
        final Trigger driverL4 = driverController.rightBumper().and(coralMode);
        driverAlgaeDescore = driverController.a().and(coralMode);
        driverLeftReef = driverController.x().and(coralMode);
        driverRightReef = driverController.b().and(coralMode).or(driverAlgaeDescore);
        final Trigger driverAnyReef = driverLeftReef.or(driverRightReef);
        //algae
        // final Trigger driverProcessor = algaeMode.and(driverController.b());
        final Trigger driverProcessor = driverController.povUp();
        final Trigger driverNet = algaeMode.and(driverController.x());
        final Trigger driverNetSlow = algaeMode.and(driverController.b());
        final Trigger driverSpitAlgae = algaeMode.and(driverSpit);
        final Trigger driverIntakeAlgae = algaeMode.and(driverController.leftTrigger());
        //climb (must be in algae mode)
        final Trigger driverReadyClimb = driverController.leftStick().and(driverController.rightStick()); // left/right stick is M1 and M2
        final Trigger driverStartClimb = driverController.start();

        // More complex triggers
        robotHasCoral = intakeCoral.hasCoral.or(elevatorArm.hasPartialCoral);
        robotHasAlgae = intakeAlgae.hasAlgae.or(elevatorArmAlgae.hadAlgae);
        final Trigger justScoredCoral = new Trigger(() -> Robot.justScoredCoral);
        final Trigger drivetrainAvailable = new Trigger(() -> drivetrain.getCurrentCommand() == drivetrain.getDefaultCommand());
        final Trigger scoringCameraDisconnected = vision.scoringCameraConnected.negate();

        final Trigger targetingL2_3 = driverL2.or(driverL3);
        final Trigger reefAlgaeTarget = driverRightReef.and(targetingL2_3);
        final Trigger targetingL4 = driverL4.or(Auto.targetingLevel(4));

        Trigger generousNearReef = drivetrain.withinTargetPoseTolerance(
            Meters.of(2.5),
            Meters.of(2.5),
            Degrees.of(45)
        );

        Trigger l1_2_3NearReef = targetingL2_3.or(driverL1).and(generousNearReef);

        nearReef = drivetrain.targetingReef().and(
                    drivetrain.withinTargetPoseTolerance(         
                        Meters.of(0.35), // 0.7 was too aggressive at orlando
                        Meters.of(0.35), // 0.7 was too aggressive at orlando
                        Degrees.of(45)
                    ).or(l1_2_3NearReef)
        );
        
        // Used for right L2 and L1, where the vision target is blocked if we deploy too early
        Trigger almostAtReef = drivetrain.targetingReef().and(drivetrain.withinTargetPoseTolerance(
                        Inches.of(3), 
                        Inches.of(3),
                        Degrees.of(4)
        ));

        atReefXY = drivetrain.withinTargetPoseTolerance(
                        Inches.of(1 * 0.75),
                        Inches.of(1 * 0.75),
                        null
        );

        atReefAngle = drivetrain.withinTargetPoseTolerance(
                        null,
                        null,
                        Degrees.of(2 * 0.75)
        );

        atReef = drivetrain.targetingReef().and(atReefXY).and(atReefAngle);

        strugglingNearReef = nearReef.and(atReef.negate()).debounce(2, DebounceType.kRising);

        if (POSING_MODE) {
            posingAtReef = new Trigger(() -> {
                return Helpers.withinTolerance(drivetrain.getPose(), vision.getClosestReefPose(), Inches.of(1 * 0.75), Inches.of(1 * 0.75), Degrees.of(2 * 0.75));
            });
        }

        // Auto triggers
        final Trigger autoCoralIntake = autonomous.and(Auto::isCoralIntaking);


        // Teleop driving
        final Function<Double, Double> axisToLinearSpeed = (axis) -> {
            axis *= DrivetrainSubsystem.MAX_SPEED;
            // Temporary slowdown logic for net scoring, until we add real driver acceleration limiting that is tied
            // to elevator height
            if (driverNet.getAsBoolean() || driverNetSlow.getAsBoolean()) axis *= 0.25;

            // Slow down a little bit when climbing or scoring L1 manually
            if (climbDeployedBool || driverController.povLeft().getAsBoolean()) axis *= 0.5;
            return axis;
         };

        final Supplier<JoystickInputs> joystickInputsSupplier = () -> {
            return inputs.of(-driverController.getLeftY(), -driverController.getLeftX())
                            .deadband(DEADBAND)
                            .polarDistanceTransform(JoystickInputs.SQUARE_KEEP_SIGN)
                            .clamp(1) // Clamp the magnitude of the inputs to 1, since polar transform can make them slightly greater than 1
                            .transform(axisToLinearSpeed);
        };

        final DoubleSupplier rotationSupplier = () -> {
            return modifyAxis(-driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_RATE;
        };
    
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, joystickInputsSupplier, rotationSupplier));
        driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope));

        // Coral reef auto-aligns
        driverLeftReef.whileTrue(new DriveToCoralPose(
            () -> vision.lastReefID,
            (tagID) -> {
                if (elevator.isTargetingReefAlgaePosition()) {
                    return vision.getReefAlgaePose(tagID, true);
                } else if (elevator.getTargetPosition() == ElevatorSubsystem.L1) {
                    return vision.getL1ReefPose(tagID, true);
                } else {
                    return vision.getReefPose(tagID, true);
                }
            }
        ).withDynamicTarget(true));

        driverRightReef.whileTrue(new DriveToCoralPose(
            () -> vision.lastReefID,
            (tagID) -> {
                if (elevator.isTargetingReefAlgaePosition()) {
                    return vision.getReefAlgaePose(tagID, false);
                } else if (elevator.getTargetPosition() == ElevatorSubsystem.L1) {
                    return vision.getL1ReefPose(tagID, false);
                } else {
                    return vision.getReefPose(tagID, false);
                }
            }
        ).withDynamicTarget(true));

        // Ensure we reset if we cancelled mid-score previously
        driverAnyReef.onTrue(Commands.runOnce(() -> Robot.currentlyScoringCoral = false));

        // Driver Coral Intake
        coralIntakeTrigger = driverIntake.and(driverSpit.negate())
                                         .or(autoCoralIntake);
        
        coralIntakeReady = coralIntakeTrigger.and(elevatorArmPivot::isAtReceivingPosition)
                                             .and(elevator::isElevatorInPosition);

        // "Drive-thru" loading directly into the arm
        // driverIntake
        //     .whileTrue(elevatorArm.intakeAndIndex().alongWith(elevator.setPosition(ElevatorSubsystem.L3), elevatorArmPivot.horizontalPosition()))
        //     .onFalse(elevator.stow().alongWith(elevatorArmPivot.receivePosition()));

        coralIntakeTrigger
            .whileTrue(elevatorArmPivot.receivePosition().alongWith(elevator.stow(), intakeCoralPivot.extend()));

        coralIntakeReady
            .whileTrue(coralIndexer.runSpeed(0.5).alongWith(elevatorArm.intakeAndIndex()));

        coralIntakeReady.and(elevatorArm.hasNoCoral)
            .whileTrue(intakeCoral.runSpeed(1));

            
        driverIntake.and(driverL1).whileTrue(intakeCoral.runSpeed(-1).alongWith(coralIndexer.runSpeed(-1), intakeCoralPivot.stow()));

        //intaking from the human player station 
        driverIntakeHP.whileTrue(elevatorArmPivot.receiveHPposition().alongWith(elevator.setPosition(ElevatorSubsystem.receiveHP), elevatorArm.reverseIntakeAndIndex()))
                        .onFalse(elevatorArmPivot.stowPosition().alongWith(elevator.stow()));

        elevatorArm.setDefaultCommand(elevatorArm.passiveIndex());
        // elevatorArm.setDefaultCommand(elevatorArm.runSpeed(1));
        elevatorArmAlgae.setDefaultCommand(elevatorArmAlgae.passiveIndex());
        
        // Notify driver we've intaken a coral
        driverIntake.and(elevatorArm.hasCoral)
            .whileTrue(new RumbleCommand(1));

        //Algae Intake (using left paddle + right trigger)
        driverIntakeAlgae.whileTrue(intakeAlgaePivot.extend().alongWith(intakeAlgae.runSpeed(1)))
                            .onFalse(intakeAlgaePivot.stow());

        driverIntakeAlgae.and(driverSpit)
            .whileTrue(intakeAlgae.runSpeed(-1));


        final Command climbDeployedRumble = new ScheduleCommand(new RumbleCommand(1).withTimeout(1));

        Command readyClimb = elevatorArmPivot.climbPosition().alongWith(
                                climber.deploy().andThen(climbDeployedRumble),
                                elevator.climbHeight(),
                                intakeCoralPivot.extremeStow(),
                                Commands.runOnce(() -> climbDeployedBool = true)
                            );

        driverReadyClimb.whileTrue(readyClimb);
        driverStartClimb.whileTrue(climber.climb());

        // Automatic processor alignment
        // driverProcessor.whileTrue(new DriveToPose(drivetrain, () -> vision.getProcessorPose(Robot.isBlue()))
        //                                 .withPoseTargetType(PoseTarget.PROCESSOR));

        // Trigger atProcessor = drivetrain.targetingProcessor()
        //                             .and(drivetrain.withinTargetPoseTolerance(
        //                                 Inches.of(1),
        //                                 Inches.of(1),
        //                                 Degrees.of(5)
        //                             ));

        // driverProcessor.and(atProcessor).and(intakeAlgae.hasAlgae).whileTrue(Commands.parallel(
        //     intakeAlgae.spit(),
        //     Commands.runOnce(() -> {
        //         if (Robot.isSimulation()) {
        //             SimLogic.outtakeAlgae();
        //         }
        //     })
        // ));

        driverProcessor.whileTrue(elevatorArmPivot.processorPosition().alongWith(
                            elevator.stow(),
                            drivetrain.targetHeadingContinuous(270.0, HeadingTarget.GYRO))
                        )
                       .onFalse(elevatorArmPivot.stowPosition().alongWith(
                            drivetrain.targetHeading(null, HeadingTarget.POSE))
                        );

        driverProcessor.and(driverSpit)
                       .and(elevatorArmPivot.isAt(ElevatorArmPivotSubsystem.PROCESSOR))
                       .and(drivetrain.withinTargetHeadingTolerance(5))
                       .whileTrue(elevatorArmAlgae.runSpeed(-0.5));

        driverProcessor.and(elevatorArmAlgae.hasAlgae.negate())
                       .whileTrue(new RumbleCommand(1));

        
        disabled.onTrue(elevatorArmPivot.coastMode());

        teleop.onTrue(Commands.sequence(
            elevatorArmPivot.brakeMode(),
            // Commands.either(
            //     Commands.none(),
            //     elevator.home(),
            //     elevator::isHomed 
            // ),
            // .horizontalPosition(),
            // Commands.either(
            //     Commands.none(),
            //     elevatorArmPivot.home().andThen(elevatorArmPivot.horizontalPosition()),
            //     elevatorArmPivot::isHomed
            // ),
            new RumbleCommand(0.5).withTimeout(0.3)
        ));

        autonomous.or(teleop).onTrue(intakeCoral.runBottomRollerSpeed(-1).withTimeout(1));

        // Stop elevator from slamming into the top if auto ends while elevator is moving up    
        autonomous.and(() -> Timer.getFPGATimestamp() - Auto.startTime >= 14.9)
                  .and(() -> !elevator.isElevatorInPosition() && elevator.getTargetErrorMeters() > 0)
                    .onTrue(
                        elevator.stow().alongWith(elevatorArmPivot.receivePosition())
                                .until(teleop.or(disabled))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                    );

        
        final Trigger manualL1 = driverController.povLeft()
                                    .and(driverAnyReef.negate())
                                    .and(elevatorArm.hasEnteringCoral.negate())
                                    .and(elevatorArm.hasPartialCoral);
        final Trigger manualL1Recently = manualL1.debounce(0.75, DebounceType.kFalling);

        // Make the robot point towards the closest side of the reef
        teleop.and(coralMode)
                .and(elevatorArm.hasPartialCoral.or(intakeCoral.hasCoral)) // EXPERIMENT
                .and(elevatorArmAlgae.hadAlgae.negate())
                .and(climbDeployed.negate())
                .and(manualL1Recently.negate())
            .whileTrue(drivetrain.targetHeadingContinuous(() -> {
                Pose2d reefPose = vision.getClosestReefPose();
                return reefPose != null ? reefPose.getRotation().getDegrees() : null;
            }, HeadingTarget.POSE));

        Command chosenElevatorHeight = elevator.run(() -> {
            // In autonomous, read the next coral scoring position from the list to determine the elevator height
            if (RobotState.isAutonomous()) {
                CoralScoringPosition next = Auto.nextCoralScoringPosition();
                if (next == null) {
                    elevator.setPositionDirect(ElevatorSubsystem.STOW);
                    return;
                }
                elevator.setPositionDirect(elevator.levelToPosition(next.level));
                return;
            }

            if (driverAlgaeDescore.getAsBoolean()) {
                int level = Field.getAlgaeLevel(drivetrain.getTargetPoseTag());
                elevator.setPositionDirect(elevator.levelToPosition(level));
                return;
            }

            if (driverL1.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L1);

            } else if (driverL2.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L2);

            } else if (driverL3.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L3);

            } else if (driverL4.getAsBoolean()) {
                elevator.setPositionDirect(ElevatorSubsystem.L4);

            } else {
                elevator.setPositionDirect(ElevatorSubsystem.STOW);
            }
        });

        Trigger autoHasCoral = autonomous.and(elevatorArm.hasPartialCoral);
        Trigger reefDeployAllowed = elevatorArm.hasEnteringCoral.negate()
                                        .and(teleop.or(autoHasCoral));
                                        

        Trigger isScoringCoral = new Trigger(() -> Robot.currentlyScoringCoral);

        Trigger rightL2 = driverRightReef.and(driverL2);
        Trigger nearReefModified = nearReef.and(rightL2.negate())
                                            .or(almostAtReef.debounce(0.1));
        
        Trigger finalReefTrigger = nearReefModified.and(reefDeployAllowed)
                                                    .or(isScoringCoral)
                                                    .or(testMode);

        finalReefTrigger
            .whileTrue(chosenElevatorHeight.alongWith(elevatorArmPivot.matchElevatorPreset()))
            .onFalse(elevator.stow().alongWith(elevatorArmPivot.receivePosition()));

        final double algaeGrabSpeed = 0.6;  // EXPERIMENT: was 0.6/0.5, sometimes dropped, 1 was too extreme

        // Algae grab from reef (start intaking algae earlier when going to descore)
        finalReefTrigger.and(driverAlgaeDescore)
            .whileTrue(elevatorArmAlgae.intakeAndIndex(algaeGrabSpeed));

        Trigger l4Advance = targetingL4.and(drivetrain.targetingReef())
                                        .and(generousNearReef)
                                        .and(finalReefTrigger.negate())
                                        .and(reefDeployAllowed)
                                        .and(elevatorArm.hasPartialCoral);

        // Trigger l4Advance = falseTrigger;

        // Move elevator partially up when approaching reef targeting L4, but not yet at
        // the range where we are near the reef
        l4Advance.whileTrue(elevator.setPosition(ElevatorSubsystem.L4_ADVANCE).alongWith(elevatorArmPivot.matchElevatorPreset()));
                // .onFalse(Commands.either(
                //     Commands.none(),
                //     elevator.stow().alongWith(elevatorArmPivot.receivePosition()),
                //     drivetrain.targetingReef()
                // ));

        // FOR TUNING - Track exit velocity of coral
        isScoringCoral.and(elevatorArm.hasNoCoral).onTrue(
            Commands.runOnce(() -> {
                SmartDashboard.putNumber("Coral " + (SimLogic.coralScored + 1) + " Eject Velocity", elevatorArm.getVelocity());
            })
        );
 
        BooleanSupplier shouldObtainAlgae = () -> {
            return elevator.isTargetingReefAlgaePosition() && 
                    driverRightReef.getAsBoolean() && 
                    driverAlgaeDescore.getAsBoolean(); 
        };

        Command obtainAlgae = elevatorArmAlgae.intakeAndIndex(algaeGrabSpeed) // ALGAE GRAB - was 1
                                              .until(elevatorArmAlgae.hasAlgae.or(driverAlgaeDescore.negate()))
                                              .withTimeout(3);

        Command scoringSequence = Commands.either(
            obtainAlgae.andThen(coralEject().deadlineFor(elevatorArmAlgae.passiveIndex())),
            coralEject(),
            shouldObtainAlgae
        );

        Trigger visionScoreReady = vision.poseEstimateDiffLow.or(scoringCameraDisconnected)
                                                             .or(() -> {
                                                                Distance elevatorTarget = elevator.getTargetPosition();
                                                                boolean blockableTarget = elevatorTarget == ElevatorSubsystem.L1 || elevatorTarget == ElevatorSubsystem.L2;
                                                                return blockableTarget && driverRightReef.getAsBoolean();
                                                             });

        // Coral scoring sequence - kCancelIncoming means nothing else will be able to stop this command until it finishes
        atReef.and(elevator.inReefPosition)
              .and(elevatorArmPivot.elevatorArmInScoringPosition)
              .and(drivetrain.almostStationary) // EXPERIMENT: remove or increase threshold
              .and(visionScoreReady).onTrue(
            Commands.sequence(
                Commands.runOnce(() -> Robot.currentlyScoringCoral = true)
                        .alongWith(drivetrain.runOnce(() -> drivetrain.drive(new ChassisSpeeds())))
                        .alongWith(scoringSequence),
                Commands.runOnce(() -> {
                    elevator.setPositionDirect(ElevatorSubsystem.STOW);
                    elevatorArmPivot.setArmPositionDirect(ElevatorArmPivotSubsystem.receiving);
                    Robot.currentlyScoringCoral = false;
                    Robot.justScoredCoral = true;

                    if (RobotState.isAutonomous()) {
                        if (!Auto.coralScoringPositions.isEmpty()) {
                            Auto.previousCoralScoringPosition = Auto.coralScoringPositions.get(0);
                            Auto.coralScoringPositions.remove(0);
                            Auto.firstCoralCycle = false;
                        }
                    }
                    SimLogic.coralScored++;
                    SmartDashboard.putNumber("Coral/" + SimLogic.coralScored, Timer.getFPGATimestamp() - Auto.startTime);
                    if (Robot.isSimulation()) {
                        SimLogic.spawnHumanPlayerCoral();
                        SimLogic.intakeHasCoral = false;
                    }
                })
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        justScoredCoral.and(teleop).onTrue(
            new RumbleCommand(1).withTimeout(0.5)
                .alongWith(Commands.runOnce(() -> Robot.justScoredCoral = false))
        );

        strugglingNearReef.and(teleop).whileTrue(
            new RumbleCommand(1)
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.2))
                .repeatedly()
        );


        // final Trigger manualL1 = driverL1.and(driverAnyReef.negate());
        final Trigger manualL1Ready = elevator.elevatorInScoringPosition.and(elevatorArmPivot.elevatorArmInScoringPosition);
        final Command manualL1Score = l1CoralEject().andThen(Commands.runOnce(() -> Robot.justScoredCoral = true));

        manualL1.whileTrue(elevator.setPosition(ElevatorSubsystem.L1).alongWith(elevatorArmPivot.matchElevatorPreset()))
                //.onFalse(elevator.stow().alongWith(elevatorArmPivot.receivePosition()));
                .onFalse(Commands.either(
                    manualL1Score,
                    Commands.none(),
                    manualL1Ready
                ).andThen(elevator.stow().alongWith(elevatorArmPivot.receivePosition())));

        // Auto-score when holding manual l1
        // manualL1.and(manualL1Ready).onTrue(manualL1Score);

        //scoring algae net
        //if arm does not have algae already (passing from algae intake to algae arm)
        // driverNet.and(intakeAlgae.hasAlgae).whileTrue(
        //     Commands.parallel(
        //         elevator.setPosition(ElevatorSubsystem.L1),
        //         elevatorArmPivot.receiveAlgaePosition(),
        //         elevatorArmAlgae.runSpeed(1)
        //     )
        // );

        // Outtaking algae without trying to score it
        driverSpitAlgae.and(driverProcessor.negate())
                        .whileTrue(elevatorArmPivot.setPosition(ElevatorArmPivotSubsystem.horizontal))
                       .onFalse(elevatorArmPivot.stowPosition());

        driverSpitAlgae.and(driverProcessor.negate())
                        .and(elevatorArmPivot.isAt(ElevatorArmPivotSubsystem.horizontal))
                       .whileTrue(elevatorArmAlgae.runSpeed(-1));

       
        // Rehome elevator
        // algaeMode.and(driverController.leftStick()).whileTrue(Commands.sequence(
        //     elevator.home(),
        //     new RumbleCommand(1).withTimeout(2) 
        // ));

        // Rehome arm
        algaeMode.and(driverController.rightStick()).whileTrue(Commands.sequence(
            Commands.runOnce(() -> elevatorArmPivot.syncAbsolute()),
            new RumbleCommand(1).withTimeout(2) 
        ));

        if (Robot.isReal()) {
            algaeMode.whileTrue(drivetrain.targetHeadingContinuous(0.0, HeadingTarget.GYRO))
                     .onFalse(drivetrain.targetHeading(null, HeadingTarget.POSE));

            // Start moving to score algae in net
            driverNet.and(drivetrain.withinTargetHeadingTolerance(5).debounce(0.1)).whileTrue(
                Commands.parallel(
                    elevator.setPosition(ElevatorSubsystem.NET_THROW),
                    elevatorArmPivot.netScorePosition()
                )
            ).onFalse(elevator.stow().alongWith(elevatorArmPivot.stowPosition()));

            Command hasCoralAfterAlgaeRumble =  new RumbleCommand(1).withTimeout(2);

            // Lob algae into the net by releasing while moving the elevator 
            driverNet.and(elevatorArmPivot.isAt(ElevatorArmPivotSubsystem.netScore))
                    .and(() -> elevator.getTargetPosition() == ElevatorSubsystem.NET_THROW && elevator.getTargetErrorInches() < 12) // was 36.5 + 15
                    .onTrue(
                        elevatorArmAlgae.runSpeed(-0.8) // was -1
                                        .withTimeout(0.5) // EXPERIMENT: 0.75, 1 second was too long
                                        .andThen(elevator.runOnce(() -> {
                                            elevator.setPositionDirect(ElevatorSubsystem.STOW);
                                            if (elevatorArm.hasPartialCoralBool()) {
                                                hasCoralAfterAlgaeRumble.schedule();
                                            }
                                        })
                    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        } else {
            algaeMode.whileTrue(vision.bargeMode());

            algaeMode.and(driverNet.negate())
                     .whileTrue(drivetrain.targetHeadingContinuous(0.0, HeadingTarget.GYRO))
                     .onFalse(drivetrain.targetHeading(null, HeadingTarget.POSE));

            driverNet.whileTrue(new DriveToPose(drivetrain, () -> vision.getBargePose())
                                    .withPoseTargetType(PoseTarget.BARGE));

            Trigger nearBarge = drivetrain.targetingBarge().and(drivetrain.withinTargetPoseTolerance(
                Meters.of(1.5), 
                Meters.of(1.5),
                Degrees.of(5)
            ));

            Trigger atBarge = drivetrain.targetingBarge().and(drivetrain.withinTargetPoseTolerance(
                Inches.of(2), 
                Inches.of(2),
                Degrees.of(3)
            ));

            // driverNet.and(nearBarge)
            //          .whileTrue(
            //             elevator.setPosition(ElevatorSubsystem.NET)
            //                     .alongWith(elevatorArmPivot.netScorePosition())
            //          )
            //          .onFalse(netPlaceStow());

            final Command netRumble = new RumbleCommand(1).withTimeout(2);

            driverNet.and(atBarge)
                     .and(elevator.elevatorInScoringPosition)
                     .and(elevatorArmPivot.isAt(ElevatorArmPivotSubsystem.netScore))
                     .onTrue(Commands.sequence(
                        elevatorArmAlgae.runSpeed(-1).withTimeout(0.5),
                        new ScheduleCommand(drivetrain.runOnce(() -> drivetrain.drive(new ChassisSpeeds())))
                            .alongWith(new ScheduleCommand(netRumble))
                     ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        }

        driverNetSlow.and(drivetrain.withinTargetHeadingTolerance(5).debounce(0.1)).whileTrue(
            Commands.parallel(
                elevator.setPosition(ElevatorSubsystem.NET),
                elevatorArmPivot.netScorePosition()
            )
        ).onFalse(Commands.sequence(
            Commands.either(
                elevatorArmAlgae.runSpeed(-1).withTimeout(0.5),
                Commands.none(),
                elevator.elevatorInScoringPosition
            ),
            netPlaceStow()
        ));

        if (leds != null) {
            leds.setDefaultCommand(leds.run(() -> {

                // temporary test code
                // if (true) {
                //     leds.setDualAnimation(leds.blueLeftFlow, leds.blueRightFlow);
                //     return;
                // }

                if (POSING_MODE) {
                    if (elevator.isElevatorInPosition() && elevatorArmPivot.isInPosition()) {
                        leds.setAnimation(leds.greenStrobe);
                    } else {
                        leds.setSplitColor(leds.BLUE, leds.RED);
                    }
                    return;
                }

                if (!vision.isScoringCameraConnected()) {
                    leds.setAnimation(leds.rainbow);
                    return;
                }

                if (!vision.isFrontCameraConnected()) {
                    leds.setAnimation(leds.yellowFadeFast);
                    return;
                }

                // if (!vision.isBackCameraConnected()) {
                //     leds.setAnimation(leds.greenStrobe);
                //     return;
                // }

                boolean doClimbAnimation = climbDeployedBool;

                if (RobotState.isDisabled()) {
                    if (doClimbAnimation) {
                        leds.setAnimation(Robot.isBlue() ? leds.blueFlow : leds.redFlow);
                    } else if (!elevatorArmPivot.isHomed()) {
                        leds.setAnimation(leds.greenStrobe);
                    } else if (!vision.hasPoseEstimates.getAsBoolean()) {
                        leds.setAnimation(leds.yellowLarson);
                    } else if (!DriverStation.isDSAttached()) {
                        leds.setAnimation(leds.yellowFade);
                    } else {
                        leds.setAnimation(Robot.isBlue() ? leds.blueFade : leds.redFade);
                    }
                    return;
                }

                if (drivetrain.isTargetingReefPose()) {
                    leds.setAnimation(leds.purpleFade);
                    return;
                }

                if (elevatorArm.hasPartialCoralBool() && !elevatorArm.hasEnteringCoralBool()) {
                    leds.setAnimation(leds.whiteFade);
                    return;
                }

                var primaryColor = Robot.isBlue() ? leds.BLUE : leds.RED;
                var topColor = robotHasAlgae.getAsBoolean() ? leds.ALGAE : primaryColor;
                var bottomColor = primaryColor;
                // var bottomColor = elevatorArm.hasPartialCoralBool() ? leds.WHITE : primaryColor;
                if (topColor != bottomColor) {
                    leds.setSplitColor(topColor, bottomColor);
                } else {
                    Animation animation;
                    if (doClimbAnimation) {
                        animation = Robot.isBlue() ? leds.blueFlow : leds.redFlow;
                    } else {
                        animation = Robot.isBlue() ? leds.blueTwinkle : leds.redTwinkle;
                    }
                    leds.setAnimation(animation);
                }
            }).ignoringDisable(true));
        }

        // ================= Autonomous Trigger Logic =================

        // autonomous.onTrue(elevatorArmPivot.setPosition(ElevatorArmPivotSubsystem.L4_ADVANCE));

        Auto.intakingState.and(autoCoralIntake).and(intakeCoral.hasCoral)
            .onTrue(Auto.driveToReefWithCoral());

        Command coralTrackingDelay = Commands.either(
            Commands.waitSeconds(0.5),
            Commands.none(),
            () -> Auto.previousCoralScoringPosition.isFrontMiddle()
        );

        Command autoHPDrive = Auto.driveToHPStation().withDeadline(
            coralTrackingDelay.andThen(Commands.waitUntil(() -> vision.getCoralPose() != null))
        );

        Command autoIntakeCoral = Commands.sequence(
            autoHPDrive,
            Auto.driveToCoral()
                .until(intakeCoral.hasCoral) // at this point, the command gets interrupted by the auto coral intake trigger
                .alongWith(Commands.runOnce(() -> Auto.seenCoralThisCycle = true))
        ).alongWith(Auto.startCoralIntake());
                                    
        justScoredCoral.and(autonomous).and(drivetrainAvailable).onTrue(
            Commands.either(
                autoIntakeCoral,
                Commands.none(),
                () -> Auto.nextCoralScoringPosition() != null
            ).alongWith(Commands.runOnce(() -> {
                Robot.justScoredCoral = false;
                Auto.seenCoralThisCycle = false;
                vision.resetCoralDetector();
            }))
        );

        final Trigger lostCoral = new Trigger(() -> {
            return Auto.seenCoralThisCycle && vision.getCoralPose() == null;
        }).debounce(1, DebounceType.kRising);

        lostCoral.and(autonomous).and(Auto.intakingState).onTrue(Auto.retryIntake());

        if (Robot.isSimulation()) {
            strugglingNearReef.and(autonomous).onTrue(Commands.sequence(
                Commands.waitSeconds(0.3).deadlineFor(intakeCoralPivot.stow()),
                drivetrain.run(() -> drivetrain.drive(new ChassisSpeeds(0, 0, 5))).withTimeout(1.5),
                new ScheduleCommand(Auto.driveToReefWithCoral())
            ));
        }

        // ====================== TEST CONTROLS ======================

        testController.button(1).onTrue(elevator.home());

        testController.button(2).onTrue(Commands.runOnce(() -> elevatorArmPivot.syncAbsolute()));


        testController.button(6).whileTrue(elevator.stow().alongWith(elevatorArmPivot.receivePosition()));
        testController.button(7).whileTrue(elevator.setPosition(ElevatorSubsystem.L4).alongWith(elevatorArmPivot.matchElevatorPreset()));


        testController.button(8).whileTrue(elevatorArm.runSpeed(0.5));
        testController.button(9).whileTrue(elevatorArm.runSpeed(1));

        // testController.button(6).whileTrue(climber.runSpeed(0.1));
        // testController.button(7).whileTrue(climber.runSpeed(-0.1));

        // testController.button(8).whileTrue(readyClimb);
        // testController.button(9).whileTrue(readyClimb);
        // testController.button(10).whileTrue(climber.climb());


        // testController.button(1).whileTrue(intakeAlgaePivot.runSpeed(0.25));
        // testController.button(2).whileTrue(intakeAlgaePivot.runSpeed(-0.25));
        //testController.button(3).whileTrue(intakeAlgae.runSpeed(0.5));
        //testController.button(4).whileTrue(intakeAlgae.runSpeed(-0.5));
        // testController.button(3).whileTrue(intakeAlgaePivot.runWinchSpeed(0.75));

        // testController.button(4).whileTrue(elevatorArmPivot.horizontalPosition().alongWith(elevator.climbHeight()));
        // testController.button(5).whileTrue(intakeAlgaePivot.climb());

        // testController.button(5).whileTrue(Auto.driveToCoral().until(robotHasCoral));


        // testController.button(7).whileTrue(intakeCoralPivot.runSpeed(0.25));
        // testController.button(8).whileTrue(intakeCoralPivot.runSpeed(-0.25));

        // testController.button(9).whileTrue(intakeCoral.runBottomRollerSpeed(0.5));
        // testController.button(10).whileTrue(intakeCoral.runBottomRollerSpeed(-0.5));


        // testController.button(6).whileTrue(intakeCoralPivot.zero(Degrees.of(90)));
        // testController.button(9).whileTrue(intakeCoralPivot.stow());
        // testController.button(10).whileTrue(intakeCoralPivot.extend());

        // testController.button(9).whileTrue(elevatorArmAlgae.intakeAndIndex(0.25));
        // testController.button(10).whileTrue(elevatorArmAlgae.runSpeed(-1));


        // testController.button(1).whileTrue(elevatorArm.runSpeed(1));
        // testController.button(2).whileTrue(elevatorArm.runSpeed(0.5));
        // testController.button(3).whileTrue(elevatorArm.runSpeed(0.25)); // l2 & l3 speeds


        // testController.button(1).whileTrue(intakeAlgae.setSpeed(1)).onFalse(intakeAlgae.stopIntake());
        // testController.button(2).whileTrue(intakeAlgae.setSpeed(-1)).onFalse(intakeAlgae.stopIntake());

        // testController.button(3).whileTrue(intakeAlgaePivot.setSpeed(0.2)).onFalse(intakeAlgaePivot.stop());
        // testController.button(4).whileTrue(intakeAlgaePivot.setSpeed(-0.2)).onFalse(intakeAlgaePivot.stop());
 
        // testController.button(6).onTrue(elevatorArmPivot.setPosition(elevatorArmPivot.horizontal));
        // testController.button(7).onTrue(elevatorArmPivot.setPosition(elevatorArmPivot.receiving));

        // // Stops on release since runSpeed automatically stops the motors
        // teleop.and(testController.button(8)).whileTrue(elevator.runSpeed(0.2));
        // teleop.and(testController.button(9)).whileTrue(elevator.runSpeed(-0.2));
        
        // teleop.and(testController.button(6))
        //     .onTrue(elevator.home().andThen(new RumbleCommand(0.5).withTimeout(0.5)));
        
        // teleop.and(testController.button(7))
        //     .onTrue(elevator.setPosition(ElevatorSubsystem.STOW));

        // teleop.and(testController.button(8))
        //     .onTrue(elevator.setPosition(ElevatorSubsystem.L2));

        // teleop.and(testController.button(10))
        //     .onTrue(elevator.setPosition(ElevatorSubsystem.L4));

        // teleop.and(testController.button(10))
        //     .onTrue(elevator.setPosition(ElevatorSubsystem.NET));

        // Example of using DriveToPose command + allowing the position to be influenced by the driver
        // Supplier<ChassisSpeeds> additionalSpeedsSupplier = () -> {
        //     return new ChassisSpeeds(driverController.getLeftX() * DrivetrainSubsystem.MAX_SPEED, 0, 0);
        // };
        
        // Command testDrive = new DriveToPose(drivetrain, () -> new Pose2d(14, 5, Rotation2d.fromDegrees(15)))
        //                         .withXEndTolerance(4)
        //                         .withHoldWithinTolerance(true)
        //                         .withAdditionalSpeeds(additionalSpeedsSupplier);

        // driverController.rightBumper().whileTrue(testDrive);

        
        // ================== SysId Routines ==================
        // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.

        // Elevator Sysid
        // driverController.start().and(driverController.y()).whileTrue(elevator.sysidQuasi(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(elevator.sysidQuasi(Direction.kReverse));

        // driverController.back().and(driverController.y()).whileTrue(elevator.sysidDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(elevator.sysidDynamic(Direction.kReverse));

        // Drive motor Sysid
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public boolean shouldAutoPush() {
        return pushChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        return Math.copySign(value * value, value);
    }

    private Command coralEject() {
        Command coralEject = elevatorArm.runSpeed(0.3).until(elevatorArm.hasNoCoral) // was 0.325
                                        .andThen(Commands.waitSeconds(0.2)); // could maybe be slightly less delay (0.1)?

        return Commands.select(Map.of(
                1, l1CoralEject(),
                2, l4CoralEjectFlip(),
                3, coralEject
            ), 
            () -> {
                if (elevator.getTargetPosition() == ElevatorSubsystem.L1) {
                    return 1;
                } else if (elevator.getTargetPosition() == ElevatorSubsystem.L4) {
                    return 2;
                } else {
                    return 3;
                }
            }
        );
    }

    // private Command l1CoralEject() {
    //     return elevatorArm.runSpeed(0.27).until(elevatorArm.hasNoCoral) // was .25, .30, before .15 at orlando
    //                       .andThen(Commands.waitSeconds(0.5));
    // }

    private Command l1CoralEject() {
        return elevatorArm.runSpeed(0.27).until(elevatorArm.hasNoCoral) // was .27 by end of south florida
                          .andThen(Commands.waitSeconds(0.5));
                          //.andThen(Commands.waitSeconds(0.15).deadlineFor(elevatorArmPivot.runSpeed(0.75)));
    }

    private static final double L4_EJECT_SPEED = 0.27; // was .29,  .32

    @Deprecated(since="South Florida")
    private Command l4CoralEject() {
        return elevatorArm.runSpeed(L4_EJECT_SPEED).until(elevatorArm.hasNoCoral) 
                          .andThen(Commands.waitSeconds(0.2));
    }

    // Move the arm pivot up slightly before retracting helps
    // flip bad L4 corals onto the branch
    private Command l4CoralEjectFlip() {
        return elevatorArm.runSpeed(L4_EJECT_SPEED).until(elevatorArm.hasNoCoral)
                          .andThen(Commands.waitSeconds(0.1))
                          .andThen(Commands.waitSeconds(0.15).deadlineFor(elevatorArmPivot.runSpeed(0.25))); // was .15 speed
    }

    /**
     * Stows the elevator and arm pivot after placing in the net. The arm pivot begins slightly before the elevator stow to
     * reduce the chance of the arm catching on the net if we were too close.
     */
    private Command netPlaceStow() {
        return elevatorArmPivot.stowPosition().alongWith(Commands.waitSeconds(0.1).andThen(elevator.stow()));
    }
}
