package frc.robot;

// Named constant values for the robot, including electrical mappings
public class Constants {

    public static final String CANIVORE = "Pseudo-Tesla";
    public static final double LOOP_TIME = 0.020;

    // =================== CAN IDs - CANivore ===================

    // public static final int GRABBER_MOTOR = 4;

    // =================== CAN IDs - RIO Bus ===================

    // public static final int INTAKE_LASERCAN = 13;

    // CAN IDs for TalonFX.
    public static final class CAN {
        public static final int CORAL_PLACER_ID = 24; // CAN ID for CoralPlacer.
    }

    // Motor configurations for CoralPlacer.
    public static final class CoralPlacerSubsystem {
        public static final double RAMP_RATE_SECONDS = 0.15; // How long it takes for the motor to go from rest to desired speed (CoralPlacer).
    }

    // Command parameters for CoralPlacementCycle.
    public static final class Commands {
        public static final double CORAL_HOLD_SPEED = -0.025; // Speed at which the motor holds the coral in place.
        public static final double CORAL_OUTTAKE_SPEED = 0.1; // Speed at which the motor pushes out the coral.
        public static final double PLACE_CORAL_DURATION_SECONDS = 1.0; // Duration for running the CoralPlacer motor at full speed.
        public static final double CORAL_PLACEMENT_DECELERATION_SECONDS = 1.0; // How long it takes for the CoralPlacer motor to return to rest after spinning.
    }
}
