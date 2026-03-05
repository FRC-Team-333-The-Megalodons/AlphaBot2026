package frc.robot.subsystems.shooter.Targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Targets;

public class TargetingIOReal implements TargetingIO {

  private Targets targets;
  private final InterpolatingDoubleTreeMap distanceToTOF;
  private final InterpolatingDoubleTreeMap distanceToVelocityScalar;
  private final double dragConstant;
  static double lastTargetYawVelocityRadPerSec;
  private String currentTargetName = "hub";

  public TargetingIOReal() {
    targets = new Targets();

    distanceToTOF = new InterpolatingDoubleTreeMap();
    distanceToVelocityScalar = new InterpolatingDoubleTreeMap();
    configureInterpolations();

    dragConstant = 1.65;
    lastTargetYawVelocityRadPerSec = 0;
  }

  private void configureInterpolations() {
    distanceToTOF.put(1.57, 0.8);
    distanceToTOF.put(2.00, 3.7);
    distanceToTOF.put(2.50, 3.77);
    distanceToTOF.put(3.0, 4.03);
    distanceToTOF.put(3.50, 1.1);
    distanceToTOF.put(4.0, 1.1);

    distanceToVelocityScalar.put(1.57, 0.4);
    distanceToVelocityScalar.put(2.0, 0.30);
    distanceToVelocityScalar.put(2.5, 0.25);
    distanceToVelocityScalar.put(3.00, 0.15);
    distanceToVelocityScalar.put(3.50, 0.1);
    distanceToVelocityScalar.put(4.00, 0.07);
  }

  @Override
  public void updateInputs(TargetingIOInputs inputs) {
    // No sensor inputs to update in this implementation
    inputs.targetName = currentTargetName;
  }

  private Translation2d selectTarget(String targetName) {
    return targets.select(targetName);
  }

  private Translation2d selectTarget(String targetName, Pose2d robotPose) {
    return targets.select(targetName, robotPose);
  }

  @Override
  public Translation2d getHub() {
    // 3. Update the tracked variable whenever this is called.
    // Note: Using .orElse() is safer than .get() in case Alliance isn't set yet!
    currentTargetName =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? "redHub" : "blueHub";
    return selectTarget(currentTargetName);
  }

  public Translation2d getEnemyHub() {
    currentTargetName =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? "blueHub" : "redHub";
    return selectTarget(currentTargetName);
  }

  @Override
  public Translation2d getAllianceZoneTarget(Pose2d robotPose) {
    currentTargetName =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? "redZone" : "blueZone";
    return selectTarget(currentTargetName, robotPose);
  }

  @Override
  public Translation2d getNeutralZoneTarget(Pose2d robotPose) {
    currentTargetName = "neutralZone";
    return selectTarget(currentTargetName, robotPose);
  }

  public double getDistanceFrom(Pose2d robotPose, Translation2d toTargetCoordinates) {
    return robotPose.getTranslation().getDistance(toTargetCoordinates);
  }

  public double getDistanceFromHub(Pose2d robotPose) {
    return getDistanceFrom(robotPose, getHub());
  }

  public double getDistanceFromEnemyHub(Pose2d robotPose) {
    return getDistanceFrom(robotPose, getEnemyHub());
  }

  public double getTOFFromDistance(double distanceMeters) {
    return distanceToTOF.get(distanceMeters);
  }

  public double getVelocityScalar(double distanceMeters) {
    return distanceToVelocityScalar.get(distanceMeters);
  }

  public Rotation2d getAngleTo(Pose2d robotPose, Translation2d targetCoordinates) {
    return targetCoordinates.minus(robotPose.getTranslation()).getAngle();
  }

  public Rotation2d getAngleToHub(Pose2d robotPose) {
    return getAngleTo(robotPose, getHub());
  }

  public Translation2d velocityCompensatedCoordinates(
      Pose2d robotPose, Translation2d fieldVelocity, double tof) {
    Translation2d hubOffset = getHub().minus(robotPose.getTranslation());
    double uncompensatedRange = hubOffset.getNorm();
    Rotation2d robotToGoalAngle = hubOffset.getAngle();

    Translation2d goalRelativeVelocity = fieldVelocity.rotateBy(robotToGoalAngle.unaryMinus());
    double tangentialVelocity = goalRelativeVelocity.getY();
    lastTargetYawVelocityRadPerSec = -(tangentialVelocity / uncompensatedRange);

    double velocityScalar = (1.0 - Math.exp(-dragConstant * tof)) / (dragConstant * tof);
    Translation2d goalRelativeVelocityScaled = goalRelativeVelocity.times(velocityScalar);

    double scaledRadialVelocity = goalRelativeVelocityScaled.getX();
    double scaledTangentialVelocity = goalRelativeVelocityScaled.getY();

    double baseShotSpeed = uncompensatedRange / tof;
    double effectiveShotSpeed = Math.max(baseShotSpeed - scaledRadialVelocity, 0.001);

    double angularOffsetRad = Math.atan2(-scaledTangentialVelocity, effectiveShotSpeed);

    double effectiveRange = tof * Math.hypot(scaledTangentialVelocity, effectiveShotSpeed);

    Rotation2d finalHeading = robotToGoalAngle.plus(Rotation2d.fromRadians(angularOffsetRad));
    Translation2d virtualOffset = new Translation2d(effectiveRange, finalHeading);

    return robotPose.getTranslation().plus(virtualOffset);
  }
}
