// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.spamrobotics.util.StatusSignals;
import com.spamrobotics.util.simulation.SimulatedAIRobot;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class Robot extends TimedRobot {

    private static boolean isBlueAlliance = false;

    private Command autonomousCommand;
    @NotLogged
    private final Trigger nullTrigger = new Trigger(() -> false);

    @Logged(name = "RobotContainer")
    private final RobotContainer robotContainer;

    private final Alert controllerDisconnectedAlert = new Alert("Setup - Driver controller not connected!", AlertType.kError);
    private static Robot self = null;

    public Robot() {
        robotContainer = new RobotContainer();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Epilogue.configure(config -> {
            config.minimumImportance = Logged.Importance.DEBUG;
        });
        Epilogue.bind(this);
        self = this;
    }

    @Override
    public void robotInit() {
        // Make sure Pathplanner is loaded and ready to go
        FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        robotContainer.drivetrain.clearCache();
        StatusSignals.refreshAll();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        controllerDisconnectedAlert.set(!robotContainer.driverController.isConnected());
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        // Start the selected autonomous command
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        // Make sure anything autonomous-related is stopped
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        Command currentDrive = robotContainer.drivetrain.getCurrentCommand();
        if (currentDrive != null) {
            currentDrive.cancel();
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

    // Helper methods to simplify checking if the robot is blue or red alliance
    public static boolean isBlue() {
        return isBlueAlliance;
    }

    public static boolean isRed() {
        return !isBlue();
    }

    // ========= EVERYTHING BELOW HERE IS SIMULATION CODE =========

    StructArrayPublisher<Pose3d> robotComponentPoses = NetworkTableInstance.getDefault()
                                                        .getStructArrayTopic("Robot Component Poses", Pose3d.struct)
                                                        .publish();

    @NotLogged
    Pose3d[] robotComponentPosesArray = new Pose3d[1];

    private List<SimulatedAIRobot> simulatedAIRobots = new ArrayList<>();

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
    public void simulationInit() {};

    @Override
    public void simulationPeriodic() {
        // Get the positions of all maplesim coral and publish them to NetworkTables
        Pose3d[] coral = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
        coralPoses.accept(coral);

        // Get the positions of all algae and publish them to NetworkTables
        Pose3d[] algae = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
        algaePoses.accept(algae);

        // Get the positions of all maplesim AI robots and publish them to NetworkTables
        Pose2d[] aiRobotPosesArray = new Pose2d[simulatedAIRobots.size()];
        for (int i = 0; i < simulatedAIRobots.size(); i++) {
            aiRobotPosesArray[i] = simulatedAIRobots.get(i).getPose();
        }
        aiRobotPoses.accept(aiRobotPosesArray);
    }
}
