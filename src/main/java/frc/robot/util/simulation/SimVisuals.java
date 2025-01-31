package frc.robot.util.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public abstract class SimVisuals {

    public static final Mechanism2d mech2d = new Mechanism2d(2, 2);
    private static MechanismRoot2d elevatorRoot, coralArmRoot, coralIntakeRoot;
    private static MechanismLigament2d elevatorLigament, coralArmLigament, coralIntakeLigament;

    private static final double Y_BOTTOM = 0.1; // Represents resting on the robot chassis

    private static final double ELEVATOR_BASE_HEIGHT_METERS = 1.041;
    private static final double ELEVATOR_X = 1;

    private static final double CORAL_ARM_LENGTH = 0.5;
    private static final double CORAL_ARM_Y_OFFSET = -0.7;
    private static final double CORAL_ARM_X = ELEVATOR_X + (CORAL_ARM_LENGTH / 2);

    private static final double CORAL_INTAKE_LENGTH = 0.3;
    private static final double CORAL_INTAKE_X = 0.65;

    private static final Color8Bit ELEVATOR_NOCORAL_COLOR = new Color8Bit(Color.kBlue);
    private static final Color8Bit ELEVATOR_CORAL_COLOR = new Color8Bit(Color.kGreen);

    public static void init() {
        elevatorRoot = mech2d.getRoot("Elevator Root", ELEVATOR_X, Y_BOTTOM);
        coralArmRoot = mech2d.getRoot("Coral Arm Root", CORAL_ARM_X, Y_BOTTOM);
        coralIntakeRoot = mech2d.getRoot("Coral Intake Root", CORAL_INTAKE_X, Y_BOTTOM);

        elevatorLigament = elevatorRoot.append(new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit(Color.kBlue)));
        coralArmLigament = coralArmRoot.append(new MechanismLigament2d("Coral Arm", CORAL_ARM_LENGTH, 180, 10, new Color8Bit(Color.kYellow)));
        coralIntakeLigament = coralIntakeRoot.append(new MechanismLigament2d("Coral Intake", CORAL_INTAKE_LENGTH, 90, 6, new Color8Bit(Color.kRed)));

        SmartDashboard.putData("Mechanism 2D", mech2d);
    }

    public static void setElevatorHeight(double heightMeters) {
        elevatorLigament.setLength(ELEVATOR_BASE_HEIGHT_METERS + heightMeters);
    }

    public static void setCoralIntakeDegrees(double degrees) {
        coralIntakeLigament.setAngle(degrees);
    }

    public static void setElevatorArmPivotDegrees(double degrees) {
        coralArmLigament.setAngle(degrees);
    }

    public static void update() {
        coralArmRoot.setPosition(CORAL_ARM_X, Y_BOTTOM + CORAL_ARM_Y_OFFSET + elevatorLigament.getLength());
        elevatorLigament.setColor(SimLogic.hasCoral ? ELEVATOR_CORAL_COLOR : ELEVATOR_NOCORAL_COLOR);
    }
}
