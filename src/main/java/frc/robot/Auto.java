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

    public static Command driveToHPStation() {
        return Commands.either(
            driveToHPStationFar(),
            driveToHPStationClose(),
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
            Pose2d robotPose = drivetrain.getPose();
            Pose2d coralPose = vision.getCoralPose();
            if (coralPose == null) return robotPose;

            Translation2d robotTranslation = robotPose.getTranslation();
            Translation2d coralTranslation = coralPose.getTranslation();

            // Calculate the angle in radians to the coral
            double angleToCoral = Math.atan2(coralTranslation.getY() - robotTranslation.getY(), coralTranslation.getX() - robotTranslation.getX());

            return new Pose2d(coralPose.getTranslation(), new Rotation2d(angleToCoral + Math.PI));
        }).withDynamicTarget(true);
    }

    // WIP nicer pathing to prevent arm collisions with reef or algae
    public static Function<Pose2d, Pose2d> intermediateScoringPoseSupplier = (Pose2d target) -> {
        ElevatorSubsystem elevator = RobotContainer.instance.elevator;
        Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();

        // double dist = Math.abs(robotPose.getTranslation().getDistance(target.getTranslation()));
        double offset = 0;

        // if (dist > 1.1) {
        //     offset = -0.7;
        // }

        if (!elevator.isElevatorInScoringPosition()) {
            offset = -0.1;
        }

        if (offset != 0) {
            target = target.transformBy(new Transform2d(offset, 0, Rotation2d.kZero));
        }

        return target;
    };


    public static Command driveToReefWithCoral() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> nextCoralScoringPosition().getPose())
                    .withPoseTargetType(PoseTarget.REEF)
                    .withTargetPoseTag(() -> nextCoralScoringPosition().tag)
                    .withIntermediatePoses(intermediateScoringPoseSupplier)
                    .alongWith(Auto.stopCoralIntake());
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
            drivetrain.followPath(startingPath, 0.0, false)
                .until(drivetrain.withinTargetPoseDistance(1.2))
                .andThen(new DriveToPose(drivetrain, ()-> nextCoralScoringPosition().getPose())
                            .withPoseTargetType(PoseTarget.REEF)
                            .withTargetPoseTag(() -> nextCoralScoringPosition().tag)
                            .withIntermediatePoses(intermediateScoringPoseSupplier))
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
}
