package frc.robot;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralPlacerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

@Logged
public class RobotContainer {

    // Controller inputs within this range will be set to 0, to prevent
    // accidental movement when the joystick is near the center.
    public static final double CONTROLLER_DEADBAND = 0.025;

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

        // Command rightAuto = Commands.sequence(
        //     Commands.print("Right auto start!"),
        //     Commands.runOnce(() -> {
        //         // "Pose" = x, y, rotation
        //         Pose2d startPose = new Pose2d(10.3, 7.1, new Rotation2d(0));
        //         if (Robot.isBlue()) {
        //             startPose = FlippingUtil.flipFieldPose(startPose);
        //         }
        //         drivetrain.resetPose(startPose);
        //     }),
        //     drivetrain.driveDistance(1.1),
        //     drivetrain.turnDegrees(-60),
        //     drivetrain.driveDistance(2.2),
        //     placerSubsystem.setSpeed(1).withTimeout(1.5)
        // );


        Command rightAutoV2 = Commands.sequence(
            Commands.print("Right auto start!"),
            Commands.runOnce(() -> {
                Pose2d startPose = alliancePose(10.3, 7.7, 0);
                drivetrain.resetPose(startPose);
            }),
            driveTillClose(this::getRightScoringPosition),
            placerSubsystem.scoreCoral(),
            driveTillClose(this::getRightLeavePosition)
        );

        // Set up NamedCommands used in PathPlanner
        NamedCommands.registerCommand("scoreCoral", placerSubsystem.scoreCoral());
        Command middleAuto = new PathPlannerAuto("Middle Auto");
        Command TopAuto = new PathPlannerAuto("Top Auto");

        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        // autoChooser.addOption("Blue Left Auto", Auto.getBlueLeftAuto(drivetrain, placerSubsystem, new Pose2d(7.5, 7.1, Rotation2d.fromDegrees(130))));
        autoChooser.addOption("Right Auto", rightAutoV2);
        autoChooser.addOption("Middle Auto", middleAuto);
        autoChooser.addOption("Left Auto", TopAuto);
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private Pose2d getRightScoringPosition() {
        return alliancePose(12.5, 5.25, -60);
    }

    private Pose2d getRightLeavePosition() {
        return alliancePose(16.3, 7.1, 50);
    }

    private Pose2d alliancePose(Pose2d redPose) {
        if (Robot.isBlue()) {
            return FlippingUtil.flipFieldPose(redPose);
        } else {
            return redPose;
        }
    }

    private Command driveTillClose(Supplier<Pose2d> target) {
        return drivetrain.driveTo(target).until(drivetrain.withinTargetPoseDistance(0.02));
    }

    private Pose2d alliancePose(double x, double y, double degrees) {
        Pose2d redPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        return alliancePose(redPose);
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        // Code to convert a joystick axis (-1 to 1) to a speed in meters/second
        final Function<Double, Double> axisToLinearSpeed = (axis) -> {
            return axis * DrivetrainSubsystem.MAX_SPEED;
         };

        final Supplier<JoystickInputs> joystickInputsSupplier = () -> {
            return inputs.of(-driverController.getLeftY(), -driverController.getLeftX())
                            .deadband(CONTROLLER_DEADBAND)
                            .polarDistanceTransform(JoystickInputs.SQUARE_KEEP_SIGN)
                            .clamp(1)
                            .transform(axisToLinearSpeed);
        };

        final DoubleSupplier rotationSupplier = () -> {
            return modifyAxis(-driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_RATE;
        };
    
        drivetrain.setDefaultCommand(new JoystickDriveCommand(drivetrain, joystickInputsSupplier, rotationSupplier));
        driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope));

        // Add any additional command bindings here!

        driverController.a().whileTrue(new RumbleCommand(1));

        // Coral placement at the normal speed.
        driverController.rightBumper().whileTrue(placerSubsystem.setSpeed(Constants.Commands.CORAL_OUTTAKE_SPEED));

        // Coral placement at the slower speed
        driverController.leftTrigger().whileTrue(placerSubsystem.setSpeed(Constants.Commands.CORAL_OUTTAKE_SLOW_SPEED));
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
}
