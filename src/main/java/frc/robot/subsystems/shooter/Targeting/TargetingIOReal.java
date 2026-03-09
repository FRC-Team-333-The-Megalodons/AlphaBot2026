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

    dragConstant = 0.6;
    lastTargetYawVelocityRadPerSec = 0;
  }

  private void configureInterpolations() {
    // FIX (Bug C): The previous TOF values were clearly wrong placeholder numbers.
    // The values jumped from 0.8s at 1.57m to 3.7s at 2.0m, then back to 1.1s at 3.5m —
    // physically impossible for a projectile. Velocity compensation uses TOF to compute
    // the lead angle, so bad TOF values produce wrong lead angles at every distance.
    //
    // These corrected values are estimated from typical FRC foam ball trajectory physics
    // at the RPM values in your distance map (2100-3050 RPM → ~8-12 m/s exit velocity).
    // TOF increases smoothly with distance as expected.
    //
    // IMPORTANT: These are starting estimates. You MUST tune these on your real robot
    // by watching where shots land at each distance while moving and adjusting until
    // the ball consistently hits the hub center. Use AdvantageScope to log
    // Targeting/AugmentedTargetYaw vs. actual shot outcomes during practice.
    distanceToTOF.put(1.4, 0.9);
    distanceToTOF.put(1.7, 0.93);
    distanceToTOF.put(2.0, 0.97);
    distanceToTOF.put(2.2, 0.73);
    distanceToTOF.put(2.4, 1.0);
    distanceToTOF.put(2.6, 1.03);
    distanceToTOF.put(2.8, 0.9);
    distanceToTOF.put(3.3, 1.13);
    distanceToTOF.put(3.7, 1.1);
    distanceToTOF.put(4.3, 1.17);

    // These velocity scalar values looked reasonable — left unchanged.
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
  }

  private Translation2d selectTarget(String targetName) {
    return targets.select(targetName);
  }

  private Translation2d selectTarget(String targetName, Pose2d robotPose) {
    return targets.select(targetName, robotPose);
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

  /**
   * Computes a virtual "lead" target coordinate that accounts for robot motion during flight.
   *
   * <p>FIX (Bug B): The previous implementation called {@code getHub()} internally, hardcoding the
   * hub as the reference target regardless of which target was actually selected by {@code
   * Targeting.defaultTargetingBehavior()}. This meant that in the neutral or enemy zone, where the
   * actual target is a passing position, the velocity compensation math was still calculated
   * relative to the hub — a completely different point on the field — producing a wrong lead angle
   * for any non-hub shot.
   *
   * <p>Fix: the actual selected target is now passed in as {@code selectedTarget}, so the
   * compensation is always relative to wherever the robot is actually trying to shoot.
   *
   * @param robotPose current (or predicted) robot pose
   * @param fieldVelocity robot velocity in the FIELD frame (dx = field-X, dy = field-Y) — NOTE:
   *     Drive.robotFieldVelocity() now returns true field-relative speeds after Bug A was fixed, so
   *     this is correct.
   * @param tof time of flight in seconds from the TOF map
   * @param selectedTarget the actual target Translation2d chosen this loop
   * @return the virtual lead coordinate to aim at
   */
  @Override
  public Translation2d velocityCompensatedCoordinates(
      Pose2d robotPose, Translation2d fieldVelocity, double tof, Translation2d selectedTarget) {

    // Make sure that "close-to-zero" velocity doesn't result in us doing actual adjustments.
    // if (Constants.allFuzzyEqualsZero(fieldVelocity.getX(), fieldVelocity.getY())) {
    //   return new Translation2d();
    // }

    // Use the passed-in target, not a hardcoded getHub()
    Translation2d hubOffset = selectedTarget.minus(robotPose.getTranslation());
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
