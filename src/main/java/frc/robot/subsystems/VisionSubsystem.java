package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

@Logged
public class VisionSubsystem extends SubsystemBase {
    private static final String BACK_LIMELIGHT = "back-limelight";
    public AprilTagFieldLayout aprilTagFieldLayout;

    public Pose2d test = new Pose2d(5, 5, new  Rotation2d());
    
    private List<Integer> redTags = new ArrayList<Integer>() {{
        add(1);
        add(2);
        add(3);
        add(4); 
        add(5);
        add(6);
        add(7);
        add(8);
        add(9);
        add(10);
        add(11);
    }};

    private List<Integer> blueTags = new ArrayList<Integer>() {{
        add(12);
        add(13);
        add(14);
        add(15); 
        add(16);
        add(17);
        add(18);
        add(19);
        add(20);
        add(21);
        add(22);
    }};
    
    public VisionSubsystem() {

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }
    }

    
}
