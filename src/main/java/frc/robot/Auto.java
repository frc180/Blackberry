package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToCoralPose;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.PoseTarget;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CoralScoringPosition;
import frc.robot.util.simulation.SimLogic;

public final class Auto {

    private static final Transform2d HP_STATION_TRANSFORM = new Transform2d(0, 0, Rotation2d.fromDegrees(150));
    
    private static boolean coralIntaking = false;
    public static CoralScoringPosition previousCoralScoringPosition = null; 
    public static List<CoralScoringPosition> coralScoringPositions = new ArrayList<>();

    public static final List<CoralScoringPosition> LEFT_BARGE_CORAL_POSITIONS = List.of(
        new CoralScoringPosition(20, 4, true),
        new CoralScoringPosition(19, 4, false),
        new CoralScoringPosition(19, 4, true),
        new CoralScoringPosition(18, 4, true),
        new CoralScoringPosition(18, 4, false)
    );

    private Auto() {}
    
    public static void init() {
        coralIntaking = false;
    }

    public static boolean isCoralIntaking() {
        return coralIntaking;
    }

    public static Command startCoralIntake() {
        return Commands.runOnce(() -> coralIntaking = true);
    }

    public static Command stopCoralIntake() {
        return Commands.runOnce(() -> coralIntaking = false);
    }

    public static Command smartStartCoralIntake() {
        return Commands.run(() -> {
            Pose2d coralPickup = RobotContainer.instance.vision.getCoralPickupPose();
            if (coralPickup == null) {
                return;
            }

            Pose2d robot = RobotContainer.instance.drivetrain.getPose();
            if (robot.getTranslation().getDistance(coralPickup.getTranslation()) < 2.5) {
                coralIntaking = true;
            }
        }).until(Auto::isCoralIntaking);
    }

    public static Command driveToHPStation() {
        return Commands.either(
            driveToHPStationFar(),
            driveToHPStationClose().withMaxSpeed(0.5),
            () -> {
                CoralScoringPosition previous = previousCoralScoringPosition;
                return previous != null && previous.isFarTag();
            }
        );
    }

    public static DriveToPose driveToHPStationClose() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> {
                                    Pose2d hpStation = Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose;
                                    return hpStation.transformBy(HP_STATION_TRANSFORM);
                                });
    }

    public static Command driveToHPStationFar() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        return Commands.defer(() -> {
            double sign = Robot.isBlue() ? 1 : -1;

            Pose2d hpStation = new Pose2d((Robot.isBlue() ? SimLogic.blueHPCoralPose : SimLogic.redHPCoralPose).getTranslation(), Rotation2d.kZero);
            Translation2d end = hpStation.getTranslation().plus(new Translation2d(2 * sign, -0.5 * sign));
            List<Pose2d> pathA = List.of(
                new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(90 * sign)),
                new Pose2d(end, Rotation2d.fromDegrees(0)),
                new Pose2d(end, Rotation2d.fromDegrees(160 - (Robot.isBlue() ? 180 : 0)))
            );
            return drivetrain.followPath(pathA, 0, true);
        }, Set.of(drivetrain));
    }

    public static DriveToPose driveToCoral() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        VisionSubsystem vision = RobotContainer.instance.vision;

        return new DriveToPose(drivetrain, () -> {
            Pose2d coralPose = vision.getCoralPickupPose();
            return coralPose == null ? drivetrain.getPose() : coralPose;

            // Translation2d robotTranslation = robotPose.getTranslation();
            // Translation2d coralTranslation = coralPose.getTranslation();

            // // Calculate the angle in radians to the coral
            // double angleToCoral = Math.atan2(coralTranslation.getY() - robotTranslation.getY(), coralTranslation.getX() - robotTranslation.getX());

            // return new Pose2d(coralPose.getTranslation(), new Rotation2d(angleToCoral + Math.PI));
        }).withDynamicTarget(true);
    }

    public static Command driveToReefWithCoral() {
        return driveToNextCoralPose().alongWith(
            Commands.waitUntil(RobotContainer.instance.elevatorArm.hasCoral)
                    .andThen(Auto.stopCoralIntake())
        );
    }

    /**
     * Creates a command that configures variables needed for autonomous mode.
     * The Command should be the first one to be executed at the start of autonomous.
     * @param coralScoringPositions The list of coral scoring positions to be used in autonomous
     * @param simAutoStart The starting pose for the robot in simulation
     */
    public static Command configureAuto(List<CoralScoringPosition> coralScoringPositions, Pose2d simAutoStart) {
        return Commands.runOnce(() -> {
            setCoralScoringPositions(coralScoringPositions);
            if (Robot.isSimulation()) {
                Pose2d start = simAutoStart;
                if (Robot.isRed()) start = FlippingUtil.flipFieldPose(start);
                RobotContainer.instance.drivetrain.resetPose(start);
            }
        });
    }

    public static Command bargeCoralAuto(List<CoralScoringPosition> coralScoringPositions, List<Pose2d> startingPath, Pose2d simStart) {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;

        return Commands.parallel(
            Auto.configureAuto(coralScoringPositions, simStart),
            (driveToNextCoralPose())
            // drivetrain.followPath(startingPath, 0.0, false)
            //     .until(drivetrain.withinTargetPoseDistance(1.2))
            //     .andThen(driveToNextCoralPose())
        );
    }

    public static CoralScoringPosition nextCoralScoringPosition() {
        if (coralScoringPositions.isEmpty()) {
            return null;
        }

        return coralScoringPositions.get(0);
    }

    public static void setCoralScoringPositions(CoralScoringPosition... positions) {
        setCoralScoringPositions(List.of(positions));
    }

    public static void setCoralScoringPositions(List<CoralScoringPosition> positions) {
        coralScoringPositions.clear();
        positions.forEach(position -> coralScoringPositions.add(position.getFlippedIfNeeded()));
    }

    // =========== Helper Methods ===========

    private static DriveToPose driveToNextCoralPose() {
        return new DriveToCoralPose(
            () -> nextCoralScoringPosition().tag,
            (tag) -> nextCoralScoringPosition().getPose()
        ).withIntermediatePoses(DriveToCoralPose.AVOID_BIG_DIAGONAL);
    }

}
