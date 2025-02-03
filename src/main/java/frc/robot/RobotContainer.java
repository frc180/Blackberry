// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
import frc.robot.subsystems.elevatorArmPivot.ElevatorArmPivotSubsystem;

@Logged
public class RobotContainer {
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final boolean MAPLESIM = true;

    public final static double DEADBAND = 0.025;
    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandGenericHID testController = new CommandGenericHID(1);
    private final JoystickInputs inputs = new JoystickInputs();

    private final Telemetry logger = new Telemetry(DrivetrainSubsystem.MAX_SPEED);

    @Logged(name = "Drivetrain")
    public final DrivetrainSubsystem drivetrain;

    @Logged(name = "Vision")
    public final VisionSubsystem vision;
    public final IntakeAlgaeSubsystem intakeAlgae;
    public final IntakeAlgaePivotSubsystem intakeAlgaePivot;
    @Logged(name = "Coral Intake")
    public final IntakeCoralSubsystem intakeCoral;
    @Logged(name = "Coral Intake Pivot")
    public final IntakeCoralPivotSubsystem intakeCoralPivot;
    @Logged(name = "Elevator")
    public final ElevatorSubsystem elevator;
    @Logged(name = "Elevator ArmP")
    public final ElevatorArmPivotSubsystem elevatorArmPivot;
    @Logged(name = "Elevator arm")
    public final ElevatorArmSubsystem elevatorArm;

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public static RobotContainer instance;
    public Pose2d backLeftReefPose;

    //auto path
    public List<Waypoint> leftBargeToLeftReef;

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

        List<CoralScoringPosition> sampleAutoPositions = List.of(
            new CoralScoringPosition(20, 4, true),
            new CoralScoringPosition(19, 4, false),
            new CoralScoringPosition(19, 4, true),
            new CoralScoringPosition(18, 4, true),
            new CoralScoringPosition(18, 4, false)
        );

        backLeftReefPose = sampleAutoPositions.get(0).getPose();
        leftBargeToLeftReef = PathPlannerPath.waypointsFromPoses(
            new Pose2d(7.4, 5, Rotation2d.k180deg),
            backLeftReefPose
        );

        autoChooser.setDefaultOption("left barge to left reef", Commands.sequence(
                        Commands.runOnce(() -> {
                            Robot.setAutoCoralScoringPositions(sampleAutoPositions);
                            if (Robot.isSimulation()) {
                                Pose2d start = new Pose2d(7.4, 5, Rotation2d.k180deg);
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

        // Driver buttons
        //coral
        final Trigger driverIntake = driverController.leftTrigger();
        final Trigger driverL1 = driverController.y();
        final Trigger driverL2 = driverController.leftBumper();
        final Trigger driverL3 = driverController.rightTrigger();
        final Trigger driverL4 = driverController.rightBumper();
        final Trigger driverLeftReef = driverController.x();
        final Trigger driverRightReef = driverController.b();
        //algae
        final Trigger algaeMode = driverController.povDown();
        final Trigger driverProcessor = algaeMode.and(driverController.x());
        final Trigger driverBarge = algaeMode.and(driverController.rightBumper());
        final Trigger driverSpitAlgae = algaeMode.and(driverController.y());
        final Trigger driverIntakeAlgae = algaeMode.and(driverController.leftTrigger());
        //climb (must be in algae mode)
        final Trigger driverReadyClimb = algaeMode.and(driverController.leftStick().and(driverController.rightStick())); // left/right stick is M1 and M2
        final Trigger driverStartClimb = algaeMode.and(driverController.start());

        // More complex triggers
        final Trigger robotHasCoral = intakeCoral.hasCoral.or(elevatorArm.hasCoral);
        final Trigger justScoredCoral = new Trigger(() -> Robot.justScoredCoral);
        final Trigger drivetrainAvailable = new Trigger(() -> drivetrain.getCurrentCommand() == drivetrain.getDefaultCommand());

        // Auto triggers
        final Trigger autoCoralIntake = autonomous.and(Auto::isCoralIntaking);


        final Function<Double, Double> axisToLinearSpeed = (axis) -> {
            axis *= DrivetrainSubsystem.MAX_SPEED;
            // if (driverIntake.getAsBoolean()) axis *= 0.5;
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

        //Coral Intake (using left trigger)
        //Left Paddle = POV down
        //Right Paddle = POV up

        // Driver Coral Intake
        driverIntake.or(autoCoralIntake).and(robotHasCoral.negate())
            .whileTrue(intakeCoralPivot.setPosition(IntakeCoralPivotSubsystem.extend).alongWith(intakeCoral.intake(), elevatorArmPivot.receivePosition()))
            .onFalse(intakeCoralPivot.setPosition(IntakeCoralPivotSubsystem.stow).alongWith(intakeCoral.stopIntake()));

        // intakeCoral.hasCoral
        //     .onTrue(intakeCoralPivot.setPosition(IntakeCoralPivotSubsystem.stow).alongWith(intakeCoral.stopIntake()));
        
        //noticed that sometimes when we have a coral the intake doesnt go back to the stow position to tansfer to the arm
        intakeCoral.hasCoral.and(elevatorArmPivot.elevatorArmInPosition).and(intakeCoralPivot.atStowPosition)
            .onTrue(intakeCoral.intake().alongWith(elevatorArm.runArm()));
        
         // Notify driver we've intaken a coral
        driverIntake.and(intakeCoral.hasCoral)
            .onTrue(new RumbleCommand(1).withTimeout(0.5));

        autoCoralIntake.and(intakeCoral.hasCoral)
            .onTrue(Auto.driveToReefWithCoral());
        
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


        // Example of using the targetHeadingContinuous to make the robot point towards the closest side of the reef
        teleop.and(driverController.a().or(robotHasCoral))
            .whileTrue(drivetrain.targetHeadingContinuous(() -> {
                Pose2d reefPose = vision.getClosestReefPose();
                return reefPose != null ? reefPose.getRotation().getDegrees() : null;
            }, HeadingTarget.POSE));

        Trigger nearReef = drivetrain.targetingReef()
                            .and(drivetrain.withinTargetPoseTolerance(1.0, 1.0, 90.0))
                            .debounce(0.5, DebounceType.kFalling); 

        Trigger atReef = drivetrain.targetingReef()
                            .and(drivetrain.withinTargetPoseTolerance(0.1, 0.1, 5.0));

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

        nearReef.whileTrue(chosenElevatorHeight).onFalse(elevator.setPosition(0));


        intakeCoral.hasCoral.whileTrue(Commands.print("Coral detected!"));
        intakeCoral.doneIntaking.whileTrue(Commands.print("Done intaking!"));

        //testing the trigger - might use this to decide whether or not to score (is elevator at 0?)
        elevator.elevatorInPosition.whileTrue(Commands.print("ELEVETOR IN POSITINONNN!!!!"));
        elevator.elevatorInScoringPosition.and(elevator.elevatorInPosition).whileTrue(Commands.print("EELVATOR IS IN SCROIGN POSITITIONSS"));

        // Example scoring sequence - kCancelIncoming means nothing else will be able to stop this command until it finishes
        atReef.and(elevator.elevatorInScoringPosition).and(elevator.elevatorInPosition).onTrue(
            Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.drive(new ChassisSpeeds())),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> {
                    if (RobotState.isAutonomous()) {
                        if (!Robot.autoCoralScoringPositions.isEmpty()) {
                            Robot.autoCoralScoringPositions.remove(0);
                        }
                    }
                    Robot.justScoredCoral = true;
                    SimLogic.intakeHasCoral = false;
                    SimLogic.armHasCoral = false;
                    SimLogic.spawnHumanPlayerCoral();
                })
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        justScoredCoral.and(teleop).onTrue(
            new RumbleCommand(1).withTimeout(0.5)
                .alongWith(Commands.runOnce(() -> Robot.justScoredCoral = false))
        );


        Command autoIntakeCoral = Commands.sequence(
            Auto.driveToHPStation().until(() -> vision.getCoralPose() != null)
                                    .alongWith(Auto.coralIntake()),
            Auto.driveToCoral().until(intakeCoral.hasCoral) // at this point, the command gets interrupted by the auto coral intake trigger
        );
                                    
        justScoredCoral.and(autonomous).and(drivetrainAvailable).onTrue(
            Commands.either(
                autoIntakeCoral,
                Commands.none(),
                () -> Robot.nextAutoCoralScoringPosition() != null
            ).alongWith(Commands.runOnce(() -> Robot.justScoredCoral = false))
        );

        //using the doneIntaking & hasCoral triggers to pass on to arm


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
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
