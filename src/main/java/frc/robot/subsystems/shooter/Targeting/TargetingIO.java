package frc.robot.subsystems.shooter.Targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TargetingIO {

  @AutoLog
  public static class TargetingIOInputs {
    public String targetName = "hub";
    public double targetDistance = 0.0;
    public double targetYaw = 0.0;
    public double augmentedTargetDistance = 0.0;
    public double augmentedTargetYaw = 0.0;
  }

  public default void updateInputs(TargetingIOInputs inputs) {}

  public Translation2d getHub();

  public Translation2d getAllianceZoneTarget(Pose2d robotPose);

  public Translation2d getNeutralZoneTarget(Pose2d robotPose);

  public Rotation2d getAngleTo(Pose2d robotPose, Translation2d targetCoordinates);

  public double getDistanceFrom(Pose2d robotPose, Translation2d toTargetCoordinates);

  public double getDistanceFromHub(Pose2d robotPose);

  public double getTOFFromDistance(double distanceMeters);

  public Translation2d velocityCompensatedCoordinates(
      Pose2d robotPose, Translation2d fieldVelocity, double tof);
}
