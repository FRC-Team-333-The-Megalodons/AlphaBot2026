package frc.robot.subsystems.shooter.Targeting;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TargetingIO {

  static final Transform2d chassisToTurretOffset = new Transform2d(
    Inches.of(-6.846),
    Inches.of(-5.826),
    Rotation2d.kZero
  );

  @AutoLog
  public static class TargetingIOInputs {
    public String targetName = "hub";
    public double targetDistance = 0.0;
    public double targetYaw = 0.0;
    public double augmentedTargetDistance = 0.0;
    public double augmentedTargetYaw = 0.0;

    /**
     * The angular velocity of the target in field-frame, in radians per second. This is the rate at
     * which the field-relative angle to the (compensated) target is changing due to the robot's
     * linear motion. The turret uses this for velocity feedforward.
     */
    public double targetAngularVelocityRadPerSec = 0.0;
  }

  /**
   * Transforms the robot chassis pose into the turret pivot pose using the fixed
   * chassis-to-turret offset. All targeting math (distance, angle, velocity compensation)
   * should use this pose as the origin, since the ball launches from the turret,
   * not the robot center.
   */
  public default Pose2d getTurretPose(Pose2d robotPose) {
    return robotPose.transformBy(chassisToTurretOffset);
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

  public default Translation2d velocityCompensatedCoordinates(
      Pose2d robotPose, Translation2d fieldVelocity, double tof, Translation2d selectedTarget) {
    return new Translation2d();
  }

  public default double getLastTargetAngularVelocityRadPerSec() {
    return 0.0;
  }
}