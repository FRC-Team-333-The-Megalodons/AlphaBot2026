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

  public default Translation2d getHub() {
    return new Translation2d();
  }

  public default Translation2d getAllianceZoneTarget(Pose2d robotPose) {
    return new Translation2d();
  }

  public default Translation2d getNeutralZoneTarget(Pose2d robotPose) {
    return new Translation2d();
  }

  public default Rotation2d getAngleTo(Pose2d robotPose, Translation2d targetCoordinates) {
    return new Rotation2d();
  }

  public default double getDistanceFrom(Pose2d robotPose, Translation2d toTargetCoordinates) {
    return 0.0;
  }

  public default double getDistanceFromHub(Pose2d robotPose) {
    return 0.0;
  }

  public default double getTOFFromDistance(double distanceMeters) {
    return 0.0;
  }

  /**
   * FIX (Bug B): Added {@code selectedTarget} parameter.
   *
   * <p>Previously the signature was {@code (Pose2d, Translation2d, double)} and the implementation
   * hardcoded {@code getHub()} internally. This meant the compensation was always calculated
   * relative to the hub regardless of the actual target.
   *
   * <p>The selected target is now passed in explicitly so the math is correct for hub shots,
   * passing shots, and any future target types.
   */
  public default Translation2d velocityCompensatedCoordinates(
      Pose2d robotPose, Translation2d fieldVelocity, double tof, Translation2d selectedTarget) {
    return new Translation2d();
  }
}
