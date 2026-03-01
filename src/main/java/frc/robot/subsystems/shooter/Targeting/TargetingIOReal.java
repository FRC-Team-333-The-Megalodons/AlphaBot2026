package frc.robot.subsystems.shooter.Targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Targets;
import frc.robot.util.Zones;

public class TargetingIOReal implements TargetingIO {

  private Targets targets;

  public TargetingIOReal() {
    targets = new Targets();
  }

  @Override
  public Translation2d getHub() {
    return targets.select(
      DriverStation.getAlliance().get() == Alliance.Red ? "redHub" : "blueHub"
    );
  }

  public Translation2d getEnemyHub() {
    return targets.select(
      DriverStation.getAlliance().get() == Alliance.Red ? "blueHub" : "redHub"
    );
  }

  @Override
  public Translation2d getAllianceZoneTarget(Pose2d robotPose) {
    return targets.select(
      DriverStation.getAlliance().get() == Alliance.Red ? "redZone" : "blueZone",
      robotPose
    );
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

  /*
  public static Rotation2d getAngleToHub(Pose2d robotPose) {

  }
  */
}
