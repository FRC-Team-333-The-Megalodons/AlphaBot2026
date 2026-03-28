package frc.robot.interfaces;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.util.FlippingUtil;

public interface Posable extends Localizable {

  static final Rotation2d kCW_30deg = Rotation2d.kCW_90deg.div(3.0);
  static final Rotation2d kCW_45deg = Rotation2d.kCW_90deg.div(2.0);
  static final Rotation2d kCW_60deg = kCW_30deg.times(2.0);
  static final Rotation2d kCCW_30deg = Rotation2d.kCCW_90deg.div(3.0);
  static final Rotation2d kCCW_45deg = Rotation2d.kCCW_90deg.div(2.0);
  static final Rotation2d kCCW_60deg = kCCW_30deg.times(2.0);

  static final Distance robotXOffset = Inches.of(28).div(2);
  static final Distance robotYOffset = Inches.of(26).div(2);
  static final Distance towerWallOffset = Inches.of(41.56);
  static final Distance towerCenterOffset = Inches.of(21.0);

  static final Map<String, Pose2d> bluePoses = generatePoses();

  public default Pose2d getPose(String poseName) {
    Pose2d pose = bluePoses.get(poseName);

    if(Localizable.alliance() == Alliance.Red)
      return FlippingUtil.flipFieldPose(pose);
    else
      return pose;
  }

  public default Pose2d getPoseNear(
      Pose2d targetPose, Pose2d robotPose, Twist2d robotVelocity, double interp) {
    Translation2d predictedCoordinate =
        new Translation2d(robotPose.getX() + robotVelocity.dx, robotPose.getY() + robotVelocity.dy);

    Rotation2d predictedRotation =
        predictedCoordinate.minus(targetPose.getTranslation()).getAngle();

    Pose2d currentPose = new Pose2d(predictedCoordinate, predictedRotation);

    return currentPose.interpolate(targetPose, interp);
  }

  public default Pose2d getPoseNearCenter(Pose2d robotPose, Twist2d robotVelocity, double interp) {

    Rotation2d targetRotation;
    if (Localizable.fieldWidth().div(2).lte(Meters.of(robotPose.getY())))
      targetRotation = Rotation2d.kCW_90deg;
    else targetRotation = Rotation2d.kCCW_90deg;

    Pose2d targetPose = new Pose2d(Localizable.fieldCenter(), targetRotation);

    return getPoseNear(targetPose, robotPose, robotVelocity, interp);
  }

  private static HashMap<String, Pose2d> generatePoses() {
    HashMap<String, Pose2d> poseMap = new HashMap<>();

    bluePoses.put(
        "leftTrenchAllianceWaypoint",
        new Pose2d(
            Localizable.tagCoordinates(0).plus(Localizable.xUnitVector(-1.0)), Rotation2d.kZero));

    bluePoses.put(
        "leftTrenchNeutralWaypoint",
        new Pose2d(
            Localizable.tagCoordinates(22).plus(Localizable.xUnitVector()), Rotation2d.kZero));

    bluePoses.put(
        "rightTrenchAllianceWaypoint",
        new Pose2d(
            Localizable.tagCoordinates(28).plus(Localizable.xUnitVector(-1.0)), Rotation2d.kZero));

    bluePoses.put(
        "rightTrenchNeutralWaypoint",
        new Pose2d(
            Localizable.tagCoordinates(17).plus(Localizable.xUnitVector()), Rotation2d.kZero));

    bluePoses.put(
        "depot",
        new Pose2d(
            Translation2d.kZero
                .plus(Localizable.xUnitVector(robotXOffset.in(Meters)))
                .plus(Localizable.yUnitVector(0.75)),
            Rotation2d.k180deg));

    bluePoses.put(
        "outpost",
        new Pose2d(
            Localizable.tagCoordinates(29).plus(Localizable.xUnitVector(robotXOffset.in(Meters))),
            Rotation2d.k180deg));

    bluePoses.put(
        "leftClimb",
        new Pose2d(
            Localizable.tagCoordinates(31)
                .plus(Localizable.xUnitVector(towerWallOffset.in(Meters)))
                .plus(Localizable.yUnitVector(towerCenterOffset.in(Meters)))
                .plus(Localizable.yUnitVector(robotYOffset.in(Meters))),
            Rotation2d.k180deg));

    bluePoses.put(
        "rightClimb",
        new Pose2d(
            Localizable.tagCoordinates(31)
                .plus(Localizable.xUnitVector(towerWallOffset.in(Meters)))
                .plus(Localizable.yUnitVector(-towerCenterOffset.in(Meters)))
                .plus(Localizable.yUnitVector(-robotYOffset.in(Meters))),
            Rotation2d.kZero));

    return poseMap;
  }
}
