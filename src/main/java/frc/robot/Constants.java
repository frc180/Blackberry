package frc.robot;

// Named constant values for the robot, including electrical mappings
public class Constants {

    public static final String CANIVORE = "Pseudo-Tesla";

    public static final double LOOP_TIME = 0.020;

    //motor IDs
    public static final int ELEVATOR_REAR = 15;
    public static final int ELEVATOR_FRONT = 16;

    public static final int ELEVATOR_ARM_TALON = 19;
    public static final int ELEVATOR_ARM_PIVOT_TALON = 17;

    public static final int INTAKE_ALGAE_TALON = 18;
    public static final int INTAKE_ALGAE_PIVOT_TALON_A = 13;
    public static final int INTAKE_ALGAE_PIVOT_TALON_B = 14;

    public static final int INTAKE_CORAL_TALON = 32;
    public static final int INTAKE_CORAL_PIVOT_TALON = 33;

    public static final int ELEVATOR_ARM_ALGAE = 20;


    //sensor channels
    public static final int DIO_ELEVATOR_BOTTOM_LIMIT = 9;
    public static final int DIO_INTAKE_ALGAE_ENCODER = 1;

    public static final int INTAKE_CORAL_SENSOR = 2;
    public static final int ELEVATOR_ARM_FRONT_SENSOR = 3;
    public static final int ELEVATOR_ARM_MIDDLE_SENSOR = 4;
    public static final int ELEVATOR_ARM_BACK_SENSOR = 5;
    public static final int ELEVATOR_ARM_ALGAE_SENSOR = 6;
    public static final int INTAKE_ALGAE_SENSOR = 7;
    
}
