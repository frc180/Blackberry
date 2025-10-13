package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

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
import frc.robot.commands.CoralPlacementCycle;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralPlacerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;


@Logged
public class RobotContainer {
    /**
     * Set this to false to disable MapleSim in simulation
     */
    private static final boolean USE_MAPLESIM = true;

    public static final boolean MAPLESIM = USE_MAPLESIM && Robot.isSimulation();
    public static final double DEADBAND = 0.025;

    public final CommandXboxController driverController = new CommandXboxController(0);
    private final JoystickInputs inputs = new JoystickInputs();

    private final Telemetry logger = new Telemetry(DrivetrainSubsystem.MAX_SPEED);
    private final CoralPlacerSubsystem placerSubsystem = new CoralPlacerSubsystem();

    

    @Logged(name = "Drivetrain")
    public final DrivetrainSubsystem drivetrain;
    // Here's where you'd declare any additional subsystems, like so:
    // public final ClimberSubsystem climber;

    @NotLogged
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public static RobotContainer instance;

    public RobotContainer() {
        instance = this;

        drivetrain = TunerConstants.createDrivetrain();
        // Here's where you'd create any additional subsystems, like so:
        // climber = new ClimberSubsystem();

        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        // autoChooser.addOption("Say Hello", Commands.print("Hello, World!"));
        autoChooser.addOption("Blue Left Auto", Auto.getBlueLeftAuto(drivetrain, placerSubsystem, new Pose2d(7.5, 7.1, Rotation2d.fromDegrees(130))));

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Code to convert a joystick axis (-1 to 1) to a speed in meters/second
        final Function<Double, Double> axisToLinearSpeed = (axis) -> {
            return axis * DrivetrainSubsystem.MAX_SPEED;
         };

        final Supplier<JoystickInputs> joystickInputsSupplier = () -> {
            return inputs.of(-driverController.getLeftY(), -driverController.getLeftX())
                            .deadband(DEADBAND)
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
            driverController.rightTrigger().whileTrue(new CoralPlacementCycle(placerSubsystem, Constants.Commands.CORAL_OUTTAKE_SPEED));

            // Coral placement at the slower speed
            driverController.leftTrigger().whileTrue(new CoralPlacementCycle(placerSubsystem, Constants.Commands.CORAL_OUTTAKE_SLOW_SPEED));

        // When we're all done setting up the bindings, register telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private static double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        return Math.copySign(value * value, value);
    }
}
