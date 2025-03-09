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
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import com.pathplanner.lib.util.FlippingUtil;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
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
import frc.robot.commands.DriveToCoralPose;
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

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public Trigger robotHasCoral = new Trigger(() -> false);
    public Trigger robotHasAlgae = new Trigger(() -> false);;
    public Trigger coralIntakeTrigger = new Trigger(() -> false);
    public Trigger driverRightReef = new Trigger(() -> false);
    public Trigger driverLeftReef = new Trigger(() -> false);
    @Logged(name = "Reef - Near")
    public Trigger nearReef = null;
    @Logged(name = "Reef - At")
    public Trigger atReef = null;

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

        Pose2d firstCoralPose = Auto.LEFT_BARGE_CORAL_POSITIONS.get(0).getPose();
        List<Pose2d> leftBargeToLeftReef = List.of(
            new Pose2d(7.4, 5, Rotation2d.k180deg),
            firstCoralPose,
            firstCoralPose
        );

        Command leftBargeAuto = Auto.bargeCoralAuto(
            Auto.LEFT_BARGE_CORAL_POSITIONS,            // what positions to score at
            leftBargeToLeftReef,                        // the path to take from our starting position to the first coral position
            new Pose2d(7, 5.5, Rotation2d.fromDegrees(180 + 60))  // the position the robot should start at in simulation
        );
        if (Robot.isReal()) {
            autoChooser.setDefaultOption("Do Nothing", Commands.none());
        } else {
            autoChooser.setDefaultOption("Left Barge to Left Reef", leftBargeAuto);
            autoChooser.addOption("Do Nothing", Commands.none());
        }

        double offset = Inches.of(203).in(Meters);

        autoChooser.addOption("Drive 203 inches", Commands.sequence(
            Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(2,7, Rotation2d.kZero))),
            new DriveToPose(drivetrain, () -> new Pose2d(2 + offset, 7, Rotation2d.kZero)).until(drivetrain.withinTargetPoseDistance(0.02)),
            new DriveToPose(drivetrain, () -> new Pose2d(2, 7, Rotation2d.kZero))
        ));

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

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
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
        final Trigger driverIntake = driverController.leftTrigger().and(coralMode);
        final Trigger driverL1 = driverController.y().and(coralMode);
        final Trigger driverL2 = driverController.leftBumper().and(coralMode);
        final Trigger driverL3 = driverController.rightTrigger().and(coralMode);
        final Trigger driverL4 = driverController.rightBumper().and(coralMode);
        driverLeftReef = driverController.x().and(coralMode);
        driverRightReef = driverController.b().and(coralMode);
        Trigger driverAnyReef = driverLeftReef.or(driverRightReef);
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
        final Trigger scoringCameraDisconnected = vision.scoringCameraConnected.negate();

        Trigger reefAlgaeTarget = driverRightReef.and(driverL2.or(driverL3));

        // TODO: probably change these triggers (or where they're used) to also return true if the scoring camera is disconnected
        nearReef = drivetrain.targetingReef().and(drivetrain.withinTargetPoseTolerance(         
                        Meters.of(1),
                        Meters.of(1),
                        Degrees.of(90)
        ));//.debounce(0.2, DebounceType.kFalling); 

        Trigger almostAtReef = drivetrain.targetingReef().and(drivetrain.withinTargetPoseTolerance(
                        Inches.of(3), 
                        Inches.of(3),
                        Degrees.of(4)
        ));

        atReef = drivetrain.targetingReef().and(drivetrain.withinTargetPoseTolerance(
                        Inches.of(1), // was 1
                        Inches.of(1), // was 1
                        Degrees.of(2)
        ));

        // Auto triggers
        final Trigger autoCoralIntake = autonomous.and(Auto::isCoralIntaking);


        // Teleop driving
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
        driverController.start().whileTrue(drivetrain.wheelRadiusCharacterization(1));

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
        
        // test outtaking algae
        driverSpitAlgae.and(intakeAlgae.hasAlgae).onTrue(Commands.parallel(
            intakeAlgae.spit(),
            Commands.runOnce(() -> {
                if (Robot.isSimulation()) {
                    SimLogic.outtakeAlgae();
                }
            })
        ));

        // Driver Coral Intake
        coralIntakeTrigger = driverIntake.or(autoCoralIntake).and(elevatorArm.hasCoral.negate());

        // "Drive-thru" loading directly into the arm
        // driverIntake
        //     .whileTrue(elevatorArm.intakeAndIndex().alongWith(elevator.setPosition(ElevatorSubsystem.L3), elevatorArmPivot.horizontalPosition()))
        //     .onFalse(elevator.stow().alongWith(elevatorArmPivot.receivePosition()));

        coralIntakeTrigger
            .whileTrue(elevatorArmPivot.receivePosition().alongWith(elevator.stow(), Robot.isReal() ? Commands.none() : intakeCoralPivot.extend()));

        coralIntakeTrigger.and(elevatorArmPivot::isAtReceivingPosition).and(elevator::isElevatorInPosition)
            .whileTrue(coralIndexer.runSpeed(0.5).alongWith(elevatorArm.intakeAndIndex(), intakeCoral.runSpeed(1)));

        if (Robot.isSimulation()) {
            coralIntakeTrigger.onFalse(intakeCoralPivot.stow());
        }

        elevatorArm.setDefaultCommand(elevatorArm.passiveIndex());
        elevatorArmAlgae.setDefaultCommand(elevatorArmAlgae.passiveIndex());
        
        //noticed that sometimes when we have a coral the intake doesnt go back to the stow position to tansfer to the arm
        // intakeCoral.hasCoral.and(elevatorArmPivot.elevatorArmInPosition).and(intakeCoralPivot.atStowPosition)
        //     .onTrue(intakeCoral.intake().alongWith(elevatorArm.intakeAndIndex()));
        
        // Notify driver we've intaken a coral
        driverIntake.and(robotHasCoral)//(intakeCoral.hasCoral)
            .onTrue(new RumbleCommand(1).withTimeout(0.5));
                
        //create a trigger to check if the elevatorArm has a coral in it so that way it can stop running and go to a score/stow position
        // elevatorArm.hasCoral.onTrue(intakeCoral.stopIntake().alongWith(elevatorArm.stop()));

        //Algae Intake (using left paddle + right trigger)
        driverIntakeAlgae.whileTrue(intakeAlgaePivot.extend().alongWith(intakeAlgae.intake()))
                            .onFalse(intakeAlgaePivot.stow().alongWith(intakeAlgae.stopIntake()));

        

        //climbing sequence
        // driverReadyClimb.whileTrue(intakeAlgaePivot.readyClimb());
        driverStartClimb.whileTrue(intakeAlgaePivot.stow()); //didnt put any onFalse commands because once we climb we physically cannot un-climb

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

        // disabled.and(elevatorArmPivot.atHardstop.negate()).onTrue(
        //     elevatorArmPivot.coastMode().ignoringDisable(true)
        // );

        // disabled.and(elevatorArmPivot.atHardstop).onTrue(
        //     elevatorArmPivot.brakeMode().ignoringDisable(true)
        // );
        
        // teleop.onTrue(elevatorArmPivot.calculateAbsoluteRatio());

        // autonomous.or(teleop).onTrue(Commands.runOnce(drivetrain::zeroGyroscopeUsingPose));

        teleop.onTrue(Commands.sequence(
            elevatorArmPivot.brakeMode(),
            Commands.either(
                Commands.none(),
                elevator.home(),
                elevator::isHomed 
            ),
            // .horizontalPosition(),
            // Commands.either(
            //     Commands.none(),
            //     elevatorArmPivot.home().andThen(elevatorArmPivot.horizontalPosition()),
            //     elevatorArmPivot::isHomed
            // ),
            new RumbleCommand(0.5).withTimeout(0.3)
        ));


        // Make the robot point towards the closest side of the reef
        if (Robot.isSimulation()) {
            teleop.and(coralMode).and(robotHasCoral)
                .whileTrue(drivetrain.targetHeadingContinuous(() -> {
                    Pose2d reefPose = vision.getClosestReefPose();
                    return reefPose != null ? reefPose.getRotation().getDegrees() : null;
                }, HeadingTarget.POSE));
        }

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

        Trigger reefDeployAllowed = teleop.or(elevatorArm.hasPartialCoral);
        Trigger isScoringCoral = new Trigger(() -> Robot.currentlyScoringCoral);

        // Trigger nearReefModified = nearReef;

        Trigger rightL2 = driverRightReef.and(driverL2);
        Trigger nearReefModified = nearReef.and(rightL2.negate())
                                            .or(almostAtReef.debounce(0.1));

        // Trigger nearReefModified = nearReef.and(reefAlgaeTarget.negate())
        //                                     .or(almostAtReef.debounce(0.1));

        
        Trigger finalReefTrigger = nearReefModified.and(reefDeployAllowed).or(isScoringCoral);

        finalReefTrigger
            .whileTrue(chosenElevatorHeight.alongWith(elevatorArmPivot.matchElevatorPreset()))//, elevatorArmAlgae.intakeBasedOnElevator()))
            .onFalse(elevator.stow().alongWith(elevatorArmPivot.receivePosition()));

        // Experimental - start intaking algae earlier regardless of sensor
        // finalReefTrigger.and(reefAlgaeTarget)
        //     .whileTrue(elevatorArmAlgae.intakeAndIndex(1));

 
        BooleanSupplier shouldObtainAlgae = () -> {
            return elevator.isTargetingReefAlgaePosition() && 
                    driverRightReef.getAsBoolean() && 
                    (elevatorArmAlgae.farAlgae.getAsBoolean());// || elevatorArmAlgae.closeAlgae.getAsBoolean());
        };

        Command obtainAlgae = elevatorArmAlgae.intakeAndIndex(1)
                                              .until(elevatorArmAlgae.hasAlgae.or(driverAnyReef.negate()))
                                              .withTimeout(3);
        // Command obtainAlgae = Commands.either(
        //     elevatorArmAlgae.intakeAndIndex(0.5).until(elevatorArmAlgae.hasAlgae.or(driverAnyReef.negate())),
        //     Commands.none(),
        //     shouldObtainAlgae
        // );

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
                                                             //.or(() -> elevator.getTargetPosition() == ElevatorSubsystem.L1 && driverRightReef.getAsBoolean())
                                                             //.or(() -> elevator.getTargetPosition() == ElevatorSubsystem.L2 && driverRightReef.getAsBoolean());

        // Coral scoring sequence - kCancelIncoming means nothing else will be able to stop this command until it finishes
        atReef.and(elevator.elevatorInScoringPosition)
              .and(elevatorArmPivot.elevatorArmInScoringPosition)
              .and(drivetrain.almostStationary)
              .and(visionScoreReady).onTrue(
            Commands.sequence(
                Commands.runOnce(() -> Robot.currentlyScoringCoral = true)
                        .alongWith(drivetrain.runOnce(() -> drivetrain.drive(new ChassisSpeeds())))
                        .alongWith(scoringSequence), //.alongWith(elevatorArmEject, obtainAlgae),
                Commands.runOnce(() -> {
                    elevatorArmPivot.setArmPositionDirect(ElevatorArmPivotSubsystem.receiving);
                    Robot.currentlyScoringCoral = false;
                    Robot.justScoredCoral = true;

                    if (RobotState.isAutonomous()) {
                        if (!Auto.coralScoringPositions.isEmpty()) {
                            Auto.previousCoralScoringPosition = Auto.coralScoringPositions.get(0);
                            Auto.coralScoringPositions.remove(0);
                        }
                    }
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

        //scoring algae net
        //if arm does not have algae already (passing from algae intake to algae arm)
        // driverNet.and(intakeAlgae.hasAlgae).whileTrue(
        //     Commands.parallel(
        //         elevator.setPosition(ElevatorSubsystem.L1),
        //         elevatorArmPivot.receiveAlgaePosition(),
        //         elevatorArmAlgae.runSpeed(1)
        //     )
        // );

        // Start moving to score algae in net
        driverNet.whileTrue(
            Commands.parallel(
                elevator.setPosition(ElevatorSubsystem.NET),
                elevatorArmPivot.netScorePosition()//.alongWith(drivetrain.targetHeadingContinuous(0.0, HeadingTarget.GYRO))
                // Commands.either(
                //     elevatorArmPivot.netScorePosition().alongWith(drivetrain.targetHeadingContinuous(0.0, HeadingTarget.GYRO)),
                //     elevatorArmPivot.netScoreBackwardsPosition().alongWith(drivetrain.targetHeadingContinuous(180.0, HeadingTarget.GYRO)),
                //     () -> Math.abs(drivetrain.getGyroscopeDegreesWrapped()) <= 90
                // )
            )
        ).onFalse(Commands.sequence(
            Commands.either(
                elevatorArmAlgae.runSpeed(-1).withTimeout(0.5),
                Commands.none(),
                elevator.elevatorInPosition
            ),
            // elevatorArmAlgae.runSpeed(-1).withTimeout(0.5),
            elevator.stow().alongWith(elevatorArmPivot.stowPosition(), elevatorArmAlgae.stop())
        ));

        if (Robot.isSimulation()) {
            // Scoring algae in the net from arm
            driverNet//.and(drivetrain.withinTargetHeadingTolerance(Degrees.of(5)))
                    .and(elevator.elevatorInScoringPosition)
                    .and(elevatorArmPivot.elevatorArmInScoringPosition)
                    .and(elevatorArmAlgae.hasAlgae).whileTrue(
                        elevatorArmAlgae.runSpeed(-1)
                                        .until(elevatorArmAlgae.closeAlgae.negate().debounce(0.2))
                    );
        }

        // ================= Autonomous Trigger Logic =================

        autoCoralIntake.and(intakeCoral.hasCoral)
            .onTrue(Auto.driveToReefWithCoral());

        Command autoHPDrive = Auto.driveToHPStation().withDeadline(
            Commands.waitSeconds(0.25)
                    .andThen(Commands.waitUntil(() -> vision.getCoralPose() != null))
        );

        Command autoIntakeCoral = Commands.sequence(
            autoHPDrive,
            Auto.driveToCoral()
                .withMaxSpeed(1)
                .until(intakeCoral.hasCoral) // at this point, the command gets interrupted by the auto coral intake trigger
        ).alongWith(Auto.smartStartCoralIntake());
                                    
        justScoredCoral.and(autonomous).and(drivetrainAvailable).onTrue(
            Commands.either(
                autoIntakeCoral,
                Commands.none(),
                () -> Auto.nextCoralScoringPosition() != null
            ).alongWith(Commands.runOnce(() -> Robot.justScoredCoral = false))
        );

        // ====================== TEST CONTROLS ======================
        

        testController.button(1).whileTrue(intakeCoralPivot.runSpeed(0.50));
        testController.button(2).whileTrue(intakeCoralPivot.runSpeed(-0.50));
        testController.button(3).whileTrue(intakeCoral.runSpeed(1));
        testController.button(4).whileTrue(intakeCoral.runSpeed(-1));

        // testController.button(5).whileTrue(elevatorArmPivot.calculateAbsoluteRatio());

        // testController.button(5).whileTrue(Auto.driveToCoral());


        testController.button(6).whileTrue(intakeCoralPivot.runSpeed(0.05));
        testController.button(7).whileTrue(intakeCoralPivot.runSpeed(-0.05));


        // testController.button(6).whileTrue(intakeCoralPivot.zero(Degrees.of(90)));
        // testController.button(7).whileTrue(intakeCoralPivot.stow());
        // testController.button(8).whileTrue(intakeCoralPivot.extend());

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
        
        teleop.and(testController.button(7))
            .onTrue(elevator.setPosition(ElevatorSubsystem.STOW));

        teleop.and(testController.button(8))
            .onTrue(elevator.setPosition(ElevatorSubsystem.L2));

        teleop.and(testController.button(9))
            .onTrue(elevator.setPosition(ElevatorSubsystem.L3));

        teleop.and(testController.button(10))
            .onTrue(elevator.setPosition(ElevatorSubsystem.NET));

        // testMode.and(testController.button(10)).onTrue(Commands.sequence(
        //     elevator.home(),
        //     new RumbleCommand(0.5).withTimeout(0.5)
        // ));

        // //coral intake
        // testController.button(1).whileTrue(intakeCoral.test()).onFalse(intakeCoral.stopIntake());
        // testController.button(2).whileTrue(intakeCoralPivot.test()).onFalse(intakeCoralPivot.stop());
        // //algae intake
        // testController.button(3).whileTrue(intakeAlgae.test()).onFalse(intakeAlgae.stopIntake());
        // testController.button(4).onTrue(intakeAlgaePivot.test()).onFalse(intakeAlgaePivot.stop());
        // //elevator arm
        // testController.button(5).whileTrue(elevatorArm.test()).onFalse(elevatorArm.stop());
        // testController.button(6).whileTrue(elevatorArmAlgae.test()).onFalse(elevatorArmAlgae.stop());
        // testController.button(7).whileTrue(elevatorArmPivot.test()).onFalse(elevatorArmPivot.stop());
        // //elevator
        // testController.button(8).whileTrue(elevator.test()).onFalse(elevator.stop());

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

    private Command coralEject() {
        Command l4CoralEject = elevatorArm.runSpeed(0.425).until(elevatorArm.hasNoCoral)
                                          .andThen(Commands.waitSeconds(0.2));
        Command l1CoralEject = elevatorArm.runSpeed(0.4).until(elevatorArm.hasNoCoral);
        Command coralEject = elevatorArm.runSpeed(0.35).until(elevatorArm.hasNoCoral)
                                        .andThen(Commands.waitSeconds(0.2)); // could maybe be slightly less delay (0.1)?

        return Commands.select(Map.of(
                1, l1CoralEject,
                2, l4CoralEject,
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

}
