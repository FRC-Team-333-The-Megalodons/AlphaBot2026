package frc.robot.subsystems.shooter.Targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.interfaces.Targetable;

public class TargetingIOReal implements Targetable, TargetingIO {

  private final InterpolatingDoubleTreeMap distanceToTOF;
  private final InterpolatingDoubleTreeMap distanceToVelocityScalar;
  private final double dragConstant;

  private double lastTargetYawVelocityRadPerSec = 0.0;

  private String currentTargetName = "hub";

  public TargetingIOReal() {

    distanceToTOF = new InterpolatingDoubleTreeMap();
    distanceToVelocityScalar = new InterpolatingDoubleTreeMap();
    configureInterpolations();

    dragConstant = 0.6;
    lastTargetYawVelocityRadPerSec = 0;
  }

  private void configureInterpolations() {
    distanceToTOF.put(1.4, 0.90);
    distanceToTOF.put(1.7, 0.93);
    distanceToTOF.put(2.0, 0.97);
    distanceToTOF.put(2.2, 0.99);
    distanceToTOF.put(2.4, 1.01);
    distanceToTOF.put(2.6, 1.03);
    distanceToTOF.put(2.8, 1.06);
    distanceToTOF.put(3.3, 1.10);
    distanceToTOF.put(3.7, 1.13);
    distanceToTOF.put(4.3, 1.17);

    distanceToVelocityScalar.put(1.57, 0.4);
    distanceToVelocityScalar.put(2.0, 0.30);
    distanceToVelocityScalar.put(2.5, 0.25);
    distanceToVelocityScalar.put(3.00, 0.15);
    distanceToVelocityScalar.put(3.50, 0.1);
    distanceToVelocityScalar.put(4.00, 0.07);
  }

  @Override
  public void updateInputs(TargetingIOInputs inputs) {
    inputs.targetName = currentTargetName;
    inputs.targetAngularVelocityRadPerSec = lastTargetYawVelocityRadPerSec;
  }

  private Translation2d selectTarget(String targetName) {
    return select(targetName);
  }

  private Translation2d selectTarget(String targetName, Pose2d robotPose) {
    return select(targetName, robotPose);
  }

  @Override
  public Translation2d getHub() {
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

  @Override
  public double getDistanceFrom(Pose2d robotPose, Translation2d toTargetCoordinates) {
    return robotPose.getTranslation().getDistance(toTargetCoordinates);
  }

  @Override
  public double getDistanceFromHub(Pose2d robotPose) {
    return getDistanceFrom(robotPose, getHub());
  }

  public double getDistanceFromEnemyHub(Pose2d robotPose) {
    return getDistanceFrom(robotPose, getEnemyHub());
  }

  @Override
  public double getTOFFromDistance(double distanceMeters) {
    return distanceToTOF.get(distanceMeters);
  }

  public double getVelocityScalar(double distanceMeters) {
    return distanceToVelocityScalar.get(distanceMeters);
  }

  @Override
  public Rotation2d getAngleTo(Pose2d robotPose, Translation2d targetCoordinates) {
    return targetCoordinates.minus(robotPose.getTranslation()).getAngle();
  }

  public Rotation2d getAngleToHub(Pose2d robotPose) {
    return getAngleTo(robotPose, getHub());
  }

  @Override
  public double getLastTargetAngularVelocityRadPerSec() {
    return lastTargetYawVelocityRadPerSec;
  }

  @Override
  public Translation2d velocityCompensatedCoordinates(
      Pose2d robotPose, Translation2d fieldVelocity, double tof, Translation2d selectedTarget) {

    // Use the passed-in target, not a hardcoded getHub()

    Pose2d turretPose = getTurretPose(robotPose);

    Translation2d hubOffset = selectedTarget.minus(turretPose.getTranslation());
    double uncompensatedRange = hubOffset.getNorm();
    Rotation2d robotToGoalAngle = hubOffset.getAngle();

    Translation2d goalRelativeVelocity = fieldVelocity.rotateBy(robotToGoalAngle.unaryMinus());
    double tangentialVelocity = goalRelativeVelocity.getY();

    // This is the key value: how fast the field-relative angle to the target is changing
    // due to the robot's linear motion. Negative = clockwise angular rate.
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

    return turretPose.getTranslation().plus(virtualOffset);
  }
}
