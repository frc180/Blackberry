package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.spamrobotics.util.JoystickInputs;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralPlacerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.HeadingTarget;

@Logged
public class RobotContainer {

    // Controller inputs within this range will be set to 0, to prevent
    // accidental movement when the joystick is near the center.
    public static final double CONTROLLER_DEADBAND = 0.025;

    public static final double HP_STATION_ANGLE = 55;

    public final CommandXboxController driverController = new CommandXboxController(0);
    private final JoystickInputs inputs = new JoystickInputs();

    @Logged(name = "Drivetrain")
    public final DrivetrainSubsystem drivetrain;
    @Logged(name = "Placer")
    public final CoralPlacerSubsystem placerSubsystem;

    @NotLogged
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private final Telemetry logger = new Telemetry(DrivetrainSubsystem.MAX_SPEED);
    public static RobotContainer instance;

    public RobotContainer() {
        instance = this;

        drivetrain = TunerConstants.createDrivetrain();
        placerSubsystem = new CoralPlacerSubsystem();

        Command rightAutoV2 = Commands.sequence(
            Commands.print("Right auto start!"),
            Commands.runOnce(() -> {
                Pose2d startPose = alliancePose(10.3, 7.7, 0);
                drivetrain.resetPose(startPose);
            }),
            driveTillClose(this::getRightScoringPosition),
            placerSubsystem.scoreCoral(),
            driveTillClose(this::getRightLeavePosition, 3),
            driveTillClose(this::getRightHPPosition)
        );

        // Set up NamedCommands used in PathPlanner
        NamedCommands.registerCommand("scoreCoral", placerSubsystem.scoreCoral().asProxy());
        Command middleAuto = new PathPlannerAuto("Middle Auto");
        Command TopAuto = new PathPlannerAuto("Top Auto");

        autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
        // autoChooser.addOption("Blue Left Auto", Auto.getBlueLeftAuto(drivetrain, placerSubsystem, new Pose2d(7.5, 7.1, Rotation2d.fromDegrees(130))));
        autoChooser.addOption("Right Auto (Code)", rightAutoV2);
        autoChooser.addOption("Middle Auto (Pathplanner)", middleAuto);
        autoChooser.addOption("Left Auto (Pathplanner)", TopAuto);
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private Pose2d getRightScoringPosition() {
        return alliancePose(12.5, 5.25, -60);
    }

    private Pose2d getRightLeavePosition() {
        return alliancePose(16.3, 7.1, -60);
    }

    private Pose2d getRightHPPosition() {
        return alliancePose(16.3, 7.1, HP_STATION_ANGLE);
    }

    private Pose2d alliancePose(double x, double y, double degrees) {
        Pose2d redPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        return alliancePose(redPose);
    }

    private Pose2d alliancePose(Pose2d redPose) {
        if (Robot.isBlue()) {
            return FlippingUtil.flipFieldPose(redPose);
        } else {
            return redPose;
        }
    }

    private Command driveTillClose(Supplier<Pose2d> target) {
        return driveTillClose(target, 0.02);
    }

    private Command driveTillClose(Supplier<Pose2d> target, double withinMeters) {
        return drivetrain.driveTo(target).until(drivetrain.withinTargetPoseDistance(withinMeters));
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        final Supplier<JoystickInputs> joystickInputsSupplier = () -> {
            return inputs.of(-driverController.getLeftY(), -driverController.getLeftX())
                            .deadband(CONTROLLER_DEADBAND)
                            .polarDistanceTransform(JoystickInputs.SQUARE_KEEP_SIGN)
                            .clamp(1)
                            .transform(RobotContainer::axisToLinearSpeed);
        };

        final DoubleSupplier rotationSupplier = () -> {
            return modifyAxis(driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_RATE;
        };
    
        drivetrain.setDefaultCommand(new JoystickDriveCommand(drivetrain, joystickInputsSupplier, rotationSupplier));
        driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope));

        // Default commands run if no other command is running on the subsystem.
        // This one runs the placer backwards at a low speed to hold the coral in place.
        placerSubsystem.setDefaultCommand(placerSubsystem.setSpeed(Constants.Commands.CORAL_HOLD_SPEED));

        // Coral placement at the normal speed.
        driverController.rightTrigger().whileTrue(placerSubsystem.setSpeed(Constants.Commands.CORAL_OUTTAKE_SPEED));

        if (Robot.isSimulation()) {
            driverController.leftBumper().whileTrue(drivetrain.targetHeadingContinuous(180 - HP_STATION_ANGLE, HeadingTarget.GYRO));
            driverController.rightBumper().whileTrue(drivetrain.targetHeadingContinuous(180 + HP_STATION_ANGLE, HeadingTarget.GYRO));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Apply a deadband and square the inputs to make driving easier at low speeds.
     */
    private static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, CONTROLLER_DEADBAND);
        return Math.copySign(value * value, value);
    }

    /**
     * Convert a joystick axis (-1 to 1) to a speed in meters/second
     */
    private static double axisToLinearSpeed(double axis) {
        return axis * DrivetrainSubsystem.MAX_SPEED;
    }
}
