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
  private final InterpolatingDoubleTreeMap distanceToRPM;
  private final InterpolatingDoubleTreeMap distanceToVelocityScalar;

  public TargetingIOReal() {
    targets = new Targets();
    distanceToRPM = new InterpolatingDoubleTreeMap();
    distanceToVelocityScalar = new InterpolatingDoubleTreeMap();
    configureInterpolations();
  }

  private void configureInterpolations() {
    distanceToRPM.put(1.57, 2100.0);
    distanceToRPM.put(1.7, 2180.0);
    distanceToRPM.put(1.9, 2220.0);
    distanceToRPM.put(2.1, 2250.0);
    distanceToRPM.put(2.3, 2280.0);
    distanceToRPM.put(2.67, 2300.0);
    distanceToRPM.put(2.82, 2350.0);
    distanceToRPM.put(3.15, 2390.0);
    distanceToRPM.put(3.5, 2530.0);
    distanceToRPM.put(3.7, 2610.0);
    distanceToRPM.put(4.0, 2660.0);
    distanceToRPM.put(4.2, 2850.0);
    distanceToRPM.put(4.4, 3050.0);

    distanceToVelocityScalar.put(1.57, 0.4);
    distanceToVelocityScalar.put(2.0, 0.30);
    distanceToVelocityScalar.put(2.5, 0.25);
    distanceToVelocityScalar.put(3.00, 0.15);
    distanceToVelocityScalar.put(3.50, 0.1);
    distanceToVelocityScalar.put(4.00, 0.07);
  }

  @Override
  public Translation2d getHub() {
    return targets.select(DriverStation.getAlliance().get() == Alliance.Red ? "redHub" : "blueHub");
  }

  public Translation2d getEnemyHub() {
    return targets.select(DriverStation.getAlliance().get() == Alliance.Red ? "blueHub" : "redHub");
  }

  @Override
  public Translation2d getAllianceZoneTarget(Pose2d robotPose) {
    return targets.select(
        DriverStation.getAlliance().get() == Alliance.Red ? "redZone" : "blueZone", robotPose);
  }

  @Override
  public Translation2d getNeutralZoneTarget(Pose2d robotPose) {
    return targets.select("neutralZone", robotPose);
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

  public double getRPMFrOMDistance(double distanceMeters) {
    return distanceToRPM.get(distanceMeters);
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
  
}
