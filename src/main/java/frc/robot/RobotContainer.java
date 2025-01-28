// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.IntakeAlgae.IntakeAlgaeSubsystem;
import frc.robot.subsystems.IntakeAlgaePivot.IntakeAlgaePivotSubsystem;
import frc.robot.subsystems.IntakeCoral.IntakeCoralSubsystem;
import frc.robot.subsystems.IntakeCoralPivot.IntakeCoralPivotSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

@Logged
public class RobotContainer {
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final static double DEADBAND = 0.025;
    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandGenericHID testController = new CommandGenericHID(1);
    private final JoystickInputs inputs = new JoystickInputs();

    private final Telemetry logger = new Telemetry(DrivetrainSubsystem.MAX_SPEED);

    public final DrivetrainSubsystem drivetrain = TunerConstants.createDrivetrain();

    @Logged(name = "Vision")
    public final VisionSubsystem vision = new VisionSubsystem();
    public final IntakeAlgaeSubsystem intakeAlgae = new IntakeAlgaeSubsystem();
    public final IntakeAlgaePivotSubsystem intakeAlgaePivot = new IntakeAlgaePivotSubsystem();
    @Logged(name = "Coral Intake")
    public final IntakeCoralSubsystem intakeCoral = new IntakeCoralSubsystem();
    public final IntakeCoralPivotSubsystem intakeCoralPivot = new IntakeCoralPivotSubsystem();
    @Logged(name = "Elevator")
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    private final SendableChooser<Command> autoChooser;

    public static RobotContainer instance;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        final Trigger driverIntake = driverController.leftTrigger();
        final Trigger driverL1 = driverController.y();
        final Trigger driverL2 = driverController.leftBumper();
        final Trigger driverL3 = driverController.rightTrigger();
        final Trigger driverL4 = driverController.rightBumper();
        final Trigger driverLeftReef = driverController.x();
        final Trigger driverRightReef = driverController.b();
        final Trigger algaeMode = driverController.povDown();

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
        driverIntake.onTrue(intakeCoralPivot.setPosition(IntakeCoralPivotSubsystem.extend).alongWith(intakeCoral.intake()))
                            .onFalse(intakeCoralPivot.setPosition(IntakeCoralPivotSubsystem.stow).alongWith(intakeCoral.stopIntake()));

        //Algae Intake (using left paddle + right trigger)
        algaeMode.and(driverIntake).onTrue(intakeAlgaePivot.extend().alongWith(intakeAlgae.intake()))
                            .onFalse(intakeAlgaePivot.stow().alongWith(intakeAlgae.stopIntake()));

        //left and right alignment for the reef (x is left and b is right)
        driverLeftReef.whileTrue(new DriveToPose(drivetrain, () -> vision.getReefPose(true))
                                        .withPoseTargetType(PoseTarget.REEF));

                                        
        //testing the trigger - might use this to decide whether or not to score (is elevator at 0?)
        elevator.elevatorInPosition.whileTrue(Commands.print("ELEVETOR IN POSITINONNN!!!!"));

        driverRightReef.whileTrue(new DriveToPose(drivetrain, () -> vision.getReefPose(false))
                                        .withPoseTargetType(PoseTarget.REEF));


        // Example of using the targetHeadingContinuous to make the robot point towards the closest side of the reef
        driverController.a().whileTrue(drivetrain.targetHeadingContinuous(() -> {
            Pose2d closestReefPose = vision.getClosestReefPose();
            if (closestReefPose == null) {
                return null;
            } else {
                return closestReefPose.getRotation().getDegrees();
            }
        }, HeadingTarget.POSE));

        // Example of trigger that does something when within 0.8 meters of the reef     
        Trigger nearReef = drivetrain.targetingReef()
                            .and(drivetrain.withinTargetPoseTolerance(0.8, 0.8, 90.0))
                            .debounce(0.5, DebounceType.kFalling); 

        Command chosenElevatorHeight = elevator.run(() -> {
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

        elevator.elevatorInScoringPosition.and(elevator.elevatorInPosition).whileTrue(Commands.print("EELVATOR IS IN SCROIGN POSITITIONSS"));


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

        instance = this;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        return Math.copySign(value * value, value);
    }

    public Command plannedPath() {
        return AutoBuilder.followPath(drivetrain.path);
    }
}
