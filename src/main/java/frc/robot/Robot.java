// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.simulation.SimVisuals;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged(name = "RobotContainer")
  private final RobotContainer m_robotContainer;

  // private final boolean kUseLimelight = false;


  public Robot() {
    SimVisuals.init();

    m_robotContainer = new RobotContainer();
    Epilogue.bind(this);
  }

  @Override
  public void robotInit() {
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(16.17, 1.33, new Rotation2d())));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SimVisuals.update();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    // if (kUseLimelight) {
    //   var driveState = m_robotContainer.drivetrain.getState();
    //   double headingDeg = driveState.Pose.getRotation().getDegrees();
    //   double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    //   LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
    //   var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //   if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
    //     m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
    //   }
    // }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
        .getStructArrayTopic("CoralPoses", Pose3d.struct)
        .publish();

  @Override
  public void simulationPeriodic() {
      // Get the positions of the notes (both on the field and in the air)
      Pose3d[] coral = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
      coralPoses.accept(coral);
  }

  // Helper method to simplify checking if the robot is blue or red alliance
  public static boolean isBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public static boolean isRed() {
    return !isBlue();
  }
}


