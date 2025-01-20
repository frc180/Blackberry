package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

@Logged
public class VisionSubsystem extends SubsystemBase {

    private static final String BACK_LIMELIGHT = "limelight";
    public AprilTagFieldLayout aprilTagFieldLayout;

    private boolean odometryEnabled = false;
    private boolean backLimelightConnected = false;

    private boolean canSeeReef = false;
    public int bestReefID = -1;

    private final RawFiducial[] emptyFiducials = new RawFiducial[0];
    public RawFiducial[] rawFiducials = emptyFiducials;

    public int lastReefID;

    int[] fiducialArray = new int[0];

    private List<Integer> redTags = List.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);
    private List<Integer> blueTags = List.of(12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22);

    private List<Integer> redReefTags = List.of(6,7,8,9,10,11);
    private List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22);
    
    public VisionSubsystem() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        } else {
        }

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (Exception e) {
            aprilTagFieldLayout = null;
        }

        
    }

    @Override
    public void periodic() {

        rawFiducials = LimelightHelpers.getRawFiducials(BACK_LIMELIGHT);
        fiducialArray = new int[rawFiducials.length];
        for(int i = 0; i < rawFiducials.length; i++) {
            fiducialArray[i] = rawFiducials[i].id;
        }

        bestReefID = getReefTag(rawFiducials);
        System.out.println(bestReefID);


    }

    public int getReefTag(RawFiducial[] rawFiducial) {
        RawFiducial bestTag = null;
        List<Integer> reefTags;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            reefTags = blueReefTags;
        } else {
            reefTags = redReefTags;
        }
        // for each tag detected, look for the IDs that specifically belong to a reef based on which alliance we are on
        for(int i = 0; i< rawFiducial.length; i++) {
            RawFiducial fiducial = rawFiducial[i];
            if (reefTags.contains(fiducial.id)) {
                if (bestTag == null || bestTag.distToCamera < fiducial.distToCamera) {
                    bestTag = fiducial;
                }
            }
        }
        if (bestTag == null) {
            return -1;
        } else {
            return bestTag.id;
        }
    }

    
}
