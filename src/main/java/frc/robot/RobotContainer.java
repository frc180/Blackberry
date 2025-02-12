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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotSubsystem;
import frc.robot.subsystems.IntakeCoral.IntakeCoralSubsystem;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
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

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public Trigger robotHasCoral = null;
    public Trigger robotHasAlgae = null;
    public Trigger coralIntakeTrigger = null;
    public Trigger driverRightReef = null;
    public Trigger driverLeftReef = null;

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

        List<CoralScoringPosition> sampleAutoPositions = List.of(
            new CoralScoringPosition(20, 4, true),
            new CoralScoringPosition(19, 4, false),
            new CoralScoringPosition(19, 4, true),
            new CoralScoringPosition(18, 4, true),
            new CoralScoringPosition(18, 4, false)
        );

        Pose2d backLeftReefPose = sampleAutoPositions.get(0).getPose();
        List<Waypoint> leftBargeToLeftReef = PathPlannerPath.waypointsFromPoses(
            new Pose2d(7.4, 5, Rotation2d.k180deg),
            backLeftReefPose
        );

        autoChooser.setDefaultOption("left barge to left reef", Commands.sequence(
                        Commands.runOnce(() -> {
                            Robot.setAutoCoralScoringPositions(sampleAutoPositions);
                            if (Robot.isSimulation()) {
                                Pose2d start = new Pose2d(7.9, 5, Rotation2d.k180deg);
                                if (Robot.isRed()) start = FlippingUtil.flipFieldPose(start);
                                drivetrain.resetPose(start);
                            }

                        }),
                        drivetrain.followPath(0.0, backLeftReefPose, false, leftBargeToLeftReef)
                            .until(drivetrain.withinTargetPoseDistance(1))
                            .andThen(new DriveToPose(drivetrain, () -> Robot.nextAutoCoralScoringPosition().getPose()).withPoseTargetType(PoseTarget.REEF))
                    ));

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Robot modes
        final Trigger teleop = RobotModeTriggers.teleop();
        final Trigger autonomous = RobotModeTriggers.autonomous();
        final Trigger algaeMode = driverController.povDown();
        final Trigger coralMode = algaeMode.negate();


        // Driver buttons
        //coral
        final Trigger driverIntake = driverController.leftTrigger().and(coralMode);
        final Trigger driverL1 = driverController.y().and(coralMode);
        final Trigger driverL2 = driverController.leftBumper().and(coralMode);
        final Trigger driverL3 = driverController.rightTrigger().and(coralMode);
        final Trigger driverL4 = driverController.rightBumper().and(coralMode);
        driverLeftReef = driverController.x().and(coralMode);
        driverRightReef = driverController.b().and(coralMode);
        //algae
        final Trigger driverProcessor = algaeMode.and(driverController.b());
        final Trigger driverNet = algaeMode.and(driverController.x());
        final Trigger driverSpitAlgae = algaeMode.and(driverController.y());
        final Trigger driverIntakeAlgae = algaeMode.and(driverController.leftTrigger());
        //climb (must be in algae mode)
        final Trigger driverReadyClimb = algaeMode.and(driverController.leftStick().and(driverController.rightStick())); // left/right stick is M1 and M2
        final Trigger driverStartClimb = algaeMode.and(driverController.start());

        // More complex triggers
        robotHasCoral = intakeCoral.hasCoral.or(elevatorArm.hasCoral);
        robotHasAlgae = intakeAlgae.hasAlgae.or(elevatorArmAlgae.hasAlgae);
        final Trigger justScoredCoral = new Trigger(() -> Robot.justScoredCoral);
        final Trigger drivetrainAvailable = new Trigger(() -> drivetrain.getCurrentCommand() == drivetrain.getDefaultCommand());

        // Auto triggers
        final Trigger autoCoralIntake = autonomous.and(Auto::isCoralIntaking);


        final Function<Double, Double> axisToLinearSpeed = (axis) -> {
            axis *= DrivetrainSubsystem.MAX_SPEED;
            // Temporary slowdown logic for net scoring, until we add real driver acceleration limiting that is tied
            // to elevator height
            if (driverNet.getAsBoolean()) axis *= 0.25;
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

        // driverController.back().whileTrue(drivetrain.wheelRadiusCharacterization(-1));
        // driverController.start().whileTrue(drivetrain.wheelRadiusCharacterization(1));
        
        // test outtaking coral
        driverSpitAlgae.and(intakeAlgae.hasAlgae).onTrue(Commands.parallel(
            intakeAlgae.spit(),
            Commands.runOnce(() -> {
                if (Robot.isSimulation()) {
                    SimLogic.outtakeAlgae();
                }
            })
        ));

        // Driver Coral Intake
        coralIntakeTrigger = driverIntake.or(autoCoralIntake).and(robotHasCoral.negate());
        coralIntakeTrigger
            .whileTrue(intakeCoralPivot.setPosition(IntakeCoralPivotSubsystem.extend).alongWith(intakeCoral.intake(), elevatorArmPivot.receivePosition()))
            .onFalse(intakeCoralPivot.setPosition(IntakeCoralPivotSubsystem.stow));
        
        //noticed that sometimes when we have a coral the intake doesnt go back to the stow position to tansfer to the arm
        intakeCoral.hasCoral.and(elevatorArmPivot.elevatorArmInPosition).and(intakeCoralPivot.atStowPosition)
            .onTrue(intakeCoral.intake().alongWith(elevatorArm.runRollers()));
        
        // Notify driver we've intaken a coral
        driverIntake.and(intakeCoral.hasCoral)
            .onTrue(new RumbleCommand(1).withTimeout(0.5));
        
        // intakeCoral.doneIntaking.and(elevatorArmPivot.elevatorArmInPosition).onTrue(intakeCoral.intake().alongWith(elevatorArm.runArm()));
        
        //create a trigger to check if the elevatorArm has a coral in it so that way it can stop running and go to a score/stow position
        elevatorArm.hasCoral.onTrue(intakeCoral.stopIntake().alongWith(elevatorArm.stop(), elevatorArmPivot.stowPosition()));
        //elevatorArm.doneIntaking.onTrue(elevatorArmPivot.stowPosition());

        //Algae Intake (using left paddle + right trigger)
        //algaeMode.and(driverIntake)
        driverIntakeAlgae.whileTrue(intakeAlgaePivot.extend().alongWith(intakeAlgae.intake()))
                            .onFalse(intakeAlgaePivot.stow().alongWith(intakeAlgae.stopIntake()));

        

        //climbing sequence
        driverReadyClimb.whileTrue(intakeAlgaePivot.readyClimb());
        driverStartClimb.whileTrue(intakeAlgaePivot.stow()); //didnt put any onFalse commands because once we climb we physically cannot un-climb

        //left and right alignment for the reef (x is left and b is right)
        driverLeftReef.whileTrue(new DriveToPose(drivetrain, () -> vision.getReefPose(true))
                                        .withPoseTargetType(PoseTarget.REEF));


        driverRightReef.whileTrue(new DriveToPose(drivetrain, () -> vision.getReefPose(false))
                                        .withPoseTargetType(PoseTarget.REEF));

        //processor alignment
        driverProcessor.whileTrue(new DriveToPose(drivetrain, () -> vision.getProcessorPose(Robot.isBlue()))
                                        .withPoseTargetType(PoseTarget.PROCESSOR));

        // passing from algae arm to algae intake for processor (note: we've been told this is not possible)
        driverProcessor.and(elevatorArmAlgae.hasAlgae).whileTrue(Commands.parallel(
            elevatorArmPivot.receiveAlgaePosition(),
            elevatorArmAlgae.spit()
        ));

        Trigger atProcessor = drivetrain.targetingProcessor()
                                    .and(drivetrain.withinTargetPoseTolerance(
                                        Inches.of(1),
                                        Inches.of(1),
                                        Degrees.of(5)
                                    ));

        driverProcessor.and(atProcessor).and(intakeAlgae.hasAlgae).whileTrue(Commands.parallel(
            intakeAlgae.spit(),
            Commands.runOnce(() -> {
                if (Robot.isSimulation()) {
                    SimLogic.outtakeAlgae();
                }
            })
        ));


        // Example of using the targetHeadingContinuous to make the robot point towards the closest side of the reef
        teleop.and(coralMode).and(driverController.a().or(robotHasCoral))
            .whileTrue(drivetrain.targetHeadingContinuous(() -> {
                Pose2d reefPose = vision.getClosestReefPose();
                return reefPose != null ? reefPose.getRotation().getDegrees() : null;
            }, HeadingTarget.POSE));

        Trigger nearReef = drivetrain.targetingReef().and(drivetrain.withinTargetPoseTolerance(         
                                Meters.of(1),
                                Meters.of(1),
                                Degrees.of(90)
                            ))
                            .debounce(0.5, DebounceType.kFalling); 

        Trigger atReef = drivetrain.targetingReef().and(drivetrain.withinTargetPoseTolerance(
                                Inches.of(1),
                                Inches.of(1),
                                Degrees.of(2)
                            ));

        Command chosenElevatorHeight = elevator.run(() -> {
            // In autonomous, read the next coral scoring position from the list to determine the elevator height
            if (RobotState.isAutonomous()) {
                CoralScoringPosition next = Robot.nextAutoCoralScoringPosition();
                if (next == null) {
                    elevator.setPositionDirect(0);
                    return;
                }
                elevator.setPositionDirect(elevator.levelToPosition(next.level));
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
                elevator.setPositionDirect(0);
            }
        });

        nearReef//.and(elevatorArm.hasCoral)
            .whileTrue(chosenElevatorHeight).onFalse(elevator.setPosition(0));

        intakeCoral.hasCoral.whileTrue(Commands.print("Coral detected!"));
        intakeCoral.doneIntaking.whileTrue(Commands.print("Done intaking!"));

        //testing the trigger - might use this to decide whether or not to score (is elevator at 0?)
        elevator.elevatorInPosition.whileTrue(Commands.print("ELEVETOR IN POSITINONNN!!!!"));
        elevator.elevatorInScoringPosition.and(elevator.elevatorInPosition).whileTrue(Commands.print("EELVATOR IS IN SCROIGN POSITITIONSS"));

        Command elevatorArmEject = elevatorArm.runRollers().until(elevatorArm.hasNoCoral);
        if (Robot.isSimulation()) {
            elevatorArmEject = elevatorArmEject.withDeadline(Commands.waitSeconds(0.2));
        }

        // Coral scoring sequence - kCancelIncoming means nothing else will be able to stop this command until it finishes
        atReef.and(elevator.elevatorInScoringPosition).and(elevatorArmPivot.elevatorArmInScoringPosition).onTrue(
            Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.drive(new ChassisSpeeds())),
                elevatorArmEject,
                Commands.runOnce(() -> {
                    if (RobotState.isAutonomous()) {
                        if (!Robot.autoCoralScoringPositions.isEmpty()) {
                            Robot.autoPreviousCoralScoringPosition = Robot.autoCoralScoringPositions.get(0);
                            Robot.autoCoralScoringPositions.remove(0);
                        }
                    }
                    Robot.justScoredCoral = true;

                    if (Robot.isSimulation()) {
                        if (SimLogic.armHasCoral) SimLogic.scoreCoral();
                        SimLogic.spawnHumanPlayerCoral();
                        SimLogic.intakeHasCoral = false;
                        SimLogic.armHasCoral = false;
                    }
                })
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        justScoredCoral.and(teleop).onTrue(
            new RumbleCommand(1).withTimeout(0.5)
                .alongWith(Commands.runOnce(() -> Robot.justScoredCoral = false))
        );

        //scoring algae net
        //if arm does not have algae already (passing from algae intake to algae arm)
        driverNet.and(intakeAlgae.hasAlgae).whileTrue(
            Commands.parallel(
                elevator.setPosition(ElevatorSubsystem.L1),
                elevatorArmPivot.receiveAlgaePosition(),
                elevatorArmAlgae.run()
            )
        );

        // Start moving to score algae in net
        driverNet.and(elevatorArmAlgae.hasAlgae).whileTrue(
            Commands.parallel(
                elevator.setPosition(ElevatorSubsystem.NET),
                Commands.either(
                    elevatorArmPivot.netScorePosition().alongWith(drivetrain.targetHeadingContinuous(0.0, HeadingTarget.GYRO)),
                    elevatorArmPivot.netScoreBackwardsPosition().alongWith(drivetrain.targetHeadingContinuous(180.0, HeadingTarget.GYRO)),
                    () -> Math.abs(drivetrain.getGyroscopeDegreesWrapped()) <= 90
                )
            )
        ).onFalse(
            elevator.setPosition(0)
                .alongWith(elevatorArmPivot.stowPosition(), elevatorArmAlgae.stop())
        );

        // Scoring algae in the net from arm
        driverNet.and(drivetrain.withinTargetHeadingTolerance(Degrees.of(5)))
                 .and(elevator.elevatorInScoringPosition)
                 .and(elevatorArmPivot.elevatorArmInScoringPosition)
                 .and(elevatorArmAlgae.hasAlgae).whileTrue(
                    elevatorArmAlgae.spit()
                 );

        // ================= Autonomous Trigger Logic =================

        autoCoralIntake.and(intakeCoral.hasCoral)
            .onTrue(Auto.driveToReefWithCoral());

        Command autoHPDrive = Commands.either(
            Auto.driveToHPStationFar(),
            Auto.driveToHPStation(),
            () -> {
                CoralScoringPosition previous = Robot.autoPreviousCoralScoringPosition;
                return previous != null && previous.isFarTag();
            }
        );

        Command autoIntakeCoral = Commands.sequence(
            autoHPDrive.until(() -> vision.getCoralPose() != null)
                                    .alongWith(Auto.coralIntake()),
            Auto.driveToCoral()
                .withMaxSpeed(1)
                .until(intakeCoral.hasCoral) // at this point, the command gets interrupted by the auto coral intake trigger
        );
                                    
        justScoredCoral.and(autonomous).and(drivetrainAvailable).onTrue(
            Commands.either(
                autoIntakeCoral,
                Commands.none(),
                () -> Robot.nextAutoCoralScoringPosition() != null
            ).alongWith(Commands.runOnce(() -> Robot.justScoredCoral = false))
        );

        // ====================== TEST CONTROLS ======================

        //coral intake
        testController.button(1).onTrue(intakeCoral.test()).onFalse(intakeCoral.stopIntake());
        testController.button(2).onTrue(intakeCoralPivot.test()).onFalse(intakeCoralPivot.stop());
        //algae intake
        testController.button(3).onTrue(intakeAlgae.test()).onFalse(intakeAlgae.stopIntake());
        testController.button(4).onTrue(intakeAlgaePivot.test()).onFalse(intakeAlgaePivot.stop());
        //elevator arm
        testController.button(5).onTrue(elevatorArm.test()).onFalse(elevatorArm.stop());
        testController.button(6).onTrue(elevatorArmAlgae.test()).onFalse(elevatorArmAlgae.stop());
        testController.button(7).onTrue(elevatorArmPivot.test()).onFalse(elevatorArmPivot.stop());
        //elevator
        testController.button(8).onTrue(elevator.test()).onFalse(elevator.stop());

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

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        return Math.copySign(value * value, value);
    }

}
