package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToCoralPose;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CoralScoringPosition;
import frc.robot.util.simulation.SimLogic;

public final class Auto {

    public enum AutoState {
        IDLE,
        INTAKING,
        SCORING
    }

    private static final Transform2d LEFT_HP_STATION_TRANSFORM = new Transform2d(0, 0, Rotation2d.fromDegrees(150));
    private static final Transform2d RIGHT_HP_STATION_TRANSFORM = new Transform2d(0, 0, Rotation2d.fromDegrees(210));

    
    private static boolean coralIntaking = false;
    public static boolean leftSide = true;
    public static AutoState state = AutoState.IDLE;
    public static CoralScoringPosition previousCoralScoringPosition = null; 
    public static List<CoralScoringPosition> coralScoringPositions = new ArrayList<>();
    public static double startTime = 0;

    public static final Trigger intakingState = new Trigger(() -> state == AutoState.INTAKING);
    public static final Trigger scoringState = new Trigger(() -> state == AutoState.SCORING);

    public static List<CoralScoringPosition> leftBarge5(boolean firstBranchLeft) {
        return List.of(
            new CoralScoringPosition(20, 4, firstBranchLeft),
            new CoralScoringPosition(19, 4, false),
            new CoralScoringPosition(19, 4, true),
            new CoralScoringPosition(18, 4, true),
            new CoralScoringPosition(18, 4, false)
        );
    }

    public static List<CoralScoringPosition> leftBargeAvoidFront(boolean firstBranchLeft) {
        return List.of(
            new CoralScoringPosition(20, 4, firstBranchLeft),
            new CoralScoringPosition(19, 4, false),
            new CoralScoringPosition(19, 4, true),
            new CoralScoringPosition(20, 4, !firstBranchLeft)
        );
    }

    public static List<CoralScoringPosition> rightBarge5(boolean firstBranchLeft) {
        return List.of(
            new CoralScoringPosition(22, 4, firstBranchLeft),
            new CoralScoringPosition(17, 4, false),
            new CoralScoringPosition(17, 4, true),
            new CoralScoringPosition(18, 4, false),
            new CoralScoringPosition(18, 4, true)
        );
    }

    public static List<CoralScoringPosition> rightBargeAvoidFront(boolean firstBranchLeft) {
        return List.of(
            new CoralScoringPosition(22, 4, firstBranchLeft),
            new CoralScoringPosition(17, 4, false),
            new CoralScoringPosition(17, 4, true),
            new CoralScoringPosition(22, 4, !firstBranchLeft)
        );
    }

    public static List<CoralScoringPosition> middleBarge(boolean firstBranchLeft) {
        return List.of(
            new CoralScoringPosition(21, 4, firstBranchLeft)
        );
    }

    private Auto() {}
    
    public static void init() {
        state = AutoState.IDLE;
        coralIntaking = false;
        startTime = Timer.getFPGATimestamp();
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
            driveToHPStationClose(), // TEMPORARY speed limit
            () -> {
                CoralScoringPosition previous = previousCoralScoringPosition;
                return previous != null && previous.isFarTag();
            }
        ).alongWith(setState(AutoState.INTAKING));
    }

    private static DriveToPose driveToHPStationClose() {
        return new DriveToPose(RobotContainer.instance.drivetrain, () -> {
                                    Pose2d robotPose = RobotContainer.instance.drivetrain.getPose();

                                    Pose2d hpStation;
                                    if (Robot.isBlue()) {
                                        hpStation = leftSide ? leftBlueHPStation : rightBlueHPStation;
                                        hpStation = new Pose2d(hpStation.getTranslation(), hpStation.getRotation().rotateBy(Rotation2d.k180deg));
                                    } else {
                                        hpStation = leftSide ? leftRedHPStation : rightRedHPStation;
                                    }
                                    hpStation = hpStation.transformBy(leftSide ? LEFT_HP_STATION_TRANSFORM : RIGHT_HP_STATION_TRANSFORM);
                                    // hpStation = hpStation.transformBy(HP_STATION_TRANSFORM);
                                    // if (!leftSide) {
                                    //     hpStation = new Pose2d(hpStation.getTranslation(), hpStation.getRotation().rotateBy(Rotation2d.kCCW_90deg));
                                    // }

                                    double distance = robotPose.getTranslation().getDistance(hpStation.getTranslation());
                                    double targetDistance = distance - 1.5;
                                    Translation2d target = robotPose.getTranslation().interpolate(hpStation.getTranslation(), targetDistance / distance);
                                    return new Pose2d(target, hpStation.getRotation());
                                });
    }

    private static final Pose2d leftBlueHPStation = new Pose2d(SimLogic.blueHPCoralPose.getTranslation(), Rotation2d.kZero);
    private static final Pose2d leftRedHPStation = new Pose2d(SimLogic.redHPCoralPose.getTranslation(), Rotation2d.kZero);
    private static final Pose2d rightBlueHPStation = new Pose2d(leftBlueHPStation.getX(), FlippingUtil.fieldSizeY - leftBlueHPStation.getY(), Rotation2d.kZero);
    private static final Pose2d rightRedHPStation = new Pose2d(leftRedHPStation.getX(), FlippingUtil.fieldSizeY - leftRedHPStation.getY(), Rotation2d.kZero);
    
    private static final Translation2d leftHpStationDriveFarOffset = new Translation2d(3, -0.5);
    private static final Translation2d rightHpStationDriveFarOffset = new Translation2d(3, 0.5);

    private static Command driveToHPStationFar() {
        DrivetrainSubsystem drivetrain = RobotContainer.instance.drivetrain;
        return Commands.defer(() -> {
            double sign = (leftSide ? 1 : -1) * (Robot.isBlue() ? 1 : -1);

            // Pose2d hpStation = Robot.isBlue() ? leftBlueHPStation : leftRedHPStation;
            Pose2d hpStation;
            if (Robot.isBlue()) {
                hpStation = leftSide ? leftBlueHPStation : rightBlueHPStation;
            } else {
                hpStation = leftSide ? leftRedHPStation : rightRedHPStation;
            }
            Translation2d end;
            Translation2d offset = leftSide ? leftHpStationDriveFarOffset : rightHpStationDriveFarOffset;
            if (Robot.isBlue()) {
                end = hpStation.getTranslation().plus(offset);
            } else {
                end = hpStation.getTranslation().minus(offset);
            }
            double endAngle = 160;
            if (Robot.isBlue()) endAngle -= 180;
            List<Pose2d> pathA = List.of(
                new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(90 * sign)),
                new Pose2d(end, Rotation2d.kZero),
                new Pose2d(end, Rotation2d.fromDegrees(endAngle))
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

    private static final Distance PUSH_AMOUNT = Inches.of(15);
    private static final Distance PUSH_POSE_DIST = Inches.of(60);

    private static final double pushDistance = PUSH_POSE_DIST.in(Meters);
    private static final double pushDistanceThreshold = PUSH_POSE_DIST.minus(PUSH_AMOUNT).in(Meters);

    public static Command partnerPush() {
        var drivetrain = RobotContainer.instance.drivetrain;
        return new DriveToPose(drivetrain, () -> {
            double dir = Robot.isBlue() ? 1 : -1;
            Pose2d robotPose = drivetrain.getPose();
            return new Pose2d(robotPose.getTranslation().plus(new Translation2d(pushDistance * dir, 0)), robotPose.getRotation());
        }).until(drivetrain.withinTargetPoseDistance(pushDistanceThreshold)).withTimeout(2);
    }

    public static Command setState(AutoState newState) {
        return Commands.runOnce(() -> Auto.state = newState);
    }

    /**
     * Creates a command that configures variables needed for autonomous mode.
     * The Command should be the first one to be executed at the start of autonomous.
     * @param coralScoringPositions The list of coral scoring positions to be used in autonomous
     * @param simAutoStart The starting pose for the robot in simulation
     */
    public static Command configureAuto(List<CoralScoringPosition> coralScoringPositions, boolean left, Pose2d simAutoStart) {
        return Commands.runOnce(() -> {
            setCoralScoringPositions(coralScoringPositions);
            leftSide = left;
            if (Robot.isSimulation()) {
                Pose2d start = simAutoStart;
                if (Robot.isRed()) start = FlippingUtil.flipFieldPose(start);
                RobotContainer.instance.drivetrain.resetPose(start);
            }
        });
    }

    public static Command bargeCoralAuto(List<CoralScoringPosition> coralScoringPositions, boolean left, Pose2d simStart) {
        return Commands.parallel(
            Auto.configureAuto(coralScoringPositions, left, simStart),
            RobotContainer.instance.intakeCoralPivot.extend(),
            (driveToNextCoralPose())
        ).withName(left ? "Left Barge" : "Right Barge");
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

    private static Command driveToNextCoralPose() {
        return new DriveToCoralPose(
            () -> nextCoralScoringPosition().tag,
            (tag) -> nextCoralScoringPosition().getPose()
        ).withIntermediatePoses(DriveToCoralPose.AVOID_BIG_DIAGONAL)
        .alongWith(setState(AutoState.SCORING));
    }
}
