// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CoralScoringPosition;
import frc.robot.util.simulation.SimLogic;
import frc.robot.util.simulation.SimVisuals;
import frc.robot.util.simulation.SimulatedAIRobot;

@Logged
public class Robot extends TimedRobot {

  public static boolean justScoredCoral = false;
  public static CoralScoringPosition autoPreviousCoralScoringPosition = null; 
  public static List<CoralScoringPosition> autoCoralScoringPositions = new ArrayList<>();

  private Command m_autonomousCommand;

  @Logged(name = "RobotContainer")
  private final RobotContainer m_robotContainer;


  StructArrayPublisher<Pose3d> robotComponentPoses = NetworkTableInstance.getDefault()
                                                    .getStructArrayTopic("Robot Component Poses", Pose3d.struct)
                                                    .publish();

  Pose3d[] robotComponentPosesArray = new Pose3d[1];

  // private final boolean kUseLimelight = false;
  private List<SimulatedAIRobot> simulatedAIRobots = new ArrayList<>();

  public Robot() {
    SimVisuals.init();
    Field.init();

    m_robotContainer = new RobotContainer();
    Epilogue.bind(this);
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SimVisuals.update();

    // Get all robot component (mechanism) poses and publish them to NetworkTables
    robotComponentPosesArray[0] = RobotContainer.instance.intakeAlgaePivot.getPose();
    robotComponentPoses.accept(robotComponentPosesArray);

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
    Auto.init();
    Field.resetReefAlgae();
    if (Robot.isSimulation()) {
      SimulatedArena.getInstance().resetFieldForAuto();
      SimLogic.armHasCoral = true;
      SimLogic.intakeHasCoral = false;
      SimLogic.armHasAlgae = false;
      SimLogic.intakeHasAlgae = false;
    }
  
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
  public void teleopPeriodic() {}

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

  StructArrayPublisher<Pose2d> aiRobotPoses = NetworkTableInstance.getDefault()
          .getStructArrayTopic("AI Robot Poses", Pose2d.struct)
          .publish();

  StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
          .getStructArrayTopic("Coral Poses", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
          .getStructArrayTopic("Algae Poses", Pose3d.struct)
          .publish();

  @Override
  public void simulationInit() {
    if (RobotContainer.MAPLESIM) {
        SimLogic.spawnHumanPlayerCoral(true);
        SimLogic.spawnHumanPlayerCoral(false);
        simulatedAIRobots.add(new SimulatedAIRobot(0));
    }
  }

  private final Transform3d robotAlgaeTransform = new Transform3d(0, 0.2, 0.3, Rotation3d.kZero);

  @Override
  public void simulationPeriodic() {
      RobotContainer rc = RobotContainer.instance;
      // Get the positions of all maplesim coral and publish them to NetworkTables
      Pose3d[] coral = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
      coralPoses.accept(coral);

      // Get the positions of all algae and publish them to NetworkTables
      Pose3d[] algae = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
      Pose3d[] fieldAlgae = Field.getReefAlgaePoses();
      Pose3d robotAlgae;
      if (SimLogic.robotHasAlgae()) {
        robotAlgae = new Pose3d(rc.drivetrain.getSimPose()).transformBy(robotAlgaeTransform);
      } else {
        robotAlgae = Pose3d.kZero;
      }
      Pose3d[] combinedAlgae = new Pose3d[algae.length + fieldAlgae.length + 1];
      System.arraycopy(algae, 0, combinedAlgae, 0, algae.length);
      System.arraycopy(fieldAlgae, 0, combinedAlgae, algae.length, fieldAlgae.length);
      combinedAlgae[algae.length + fieldAlgae.length] = robotAlgae;
      algaePoses.accept(combinedAlgae);

      // Get the positions of all maplesim AI robots and publish them to NetworkTables
      Pose2d[] aiRobotPosesArray = new Pose2d[simulatedAIRobots.size()];
      for (int i = 0; i < simulatedAIRobots.size(); i++) {
          aiRobotPosesArray[i] = simulatedAIRobots.get(i).getPose();
      }
      aiRobotPoses.accept(aiRobotPosesArray);
  }

  // Helper method to simplify checking if the robot is blue or red alliance
  public static boolean isBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public static boolean isRed() {
    return !isBlue();
  }

  public static CoralScoringPosition nextAutoCoralScoringPosition() {
    if (autoCoralScoringPositions.isEmpty()) {
      return null;
    }

    return autoCoralScoringPositions.get(0);
  }

  public static void setAutoCoralScoringPositions(CoralScoringPosition... positions) {
    setAutoCoralScoringPositions(List.of(positions));
  }

  public static void setAutoCoralScoringPositions(List<CoralScoringPosition> positions) {
    autoCoralScoringPositions.clear();
    positions.forEach(position -> autoCoralScoringPositions.add(position.getFlippedIfNeeded()));
  }
}


