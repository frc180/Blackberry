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

    public static final int ELEVATOR_ARM_ALGAE = 20;

    public static final int ELEVATOR_ARM_CANDI_A = 21;
    public static final int ELEVATOR_ARM_CANDI_B = 22;

    public static final int CORAL_INDEXER = 23;

    public static final int INTAKE_CORAL_TALON = 24;
    public static final int INTAKE_CORAL_PIVOT_TALON = 25;

    public static final int INTAKE_CORAL_SPARK = 22; // RIO bus

    public static final int ALGAE_ARM_CANRANGE = 26; // unmapped

    //sensor channels
    public static final int DIO_INTAKE_ALGAE_ENCODER = 0;
    public static final int DIO_ELEVATOR_BOTTOM_LIMIT = 9;

    public static final int INTAKE_CORAL_SENSOR = 2;
    public static final int INTAKE_ALGAE_SENSOR = 7;

    public static final int ANALOG_ELEVATOR_ARM_PIVOT_POTENTIOMETER = 3;
    
}
