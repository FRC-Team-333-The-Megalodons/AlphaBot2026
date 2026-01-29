package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldLayout {
  static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  static double FIELD_WIDTH = tagLayout.getFieldWidth();
  static double FIELD_LENGTH = tagLayout.getFieldLength();

  public static class Hub {
    public static boolean isBlue =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    public static final Pose2d NEAR_FACE =
        isBlue
            ? tagLayout.getTagPose(26).get().toPose2d()
            : tagLayout.getTagPose(10).get().toPose2d();
    public static final Pose2d FAR_FACE =
        isBlue
            ? tagLayout.getTagPose(20).get().toPose2d()
            : tagLayout.getTagPose(4).get().toPose2d();
    public static final Pose2d RIGHT_FACE =
        isBlue
            ? tagLayout.getTagPose(18).get().toPose2d()
            : tagLayout.getTagPose(2).get().toPose2d();
    public static final Pose2d LEFT_FACE =
        isBlue
            ? tagLayout.getTagPose(21).get().toPose2d()
            : tagLayout.getTagPose(5).get().toPose2d();
  }

  public static class Depot {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(42.0);
    public static final double DEPTH = Units.inchesToMeters(27.0);
    public static final double HEIGHT = Units.inchesToMeters(1.125);
    public static final double DISTANCE_FROM_CENTER_Y = Units.inchesToMeters(75.93);
    // Reference Points
    public static final Translation2d DEPOT_CENTER =
        new Translation2d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y);
  }

  public static class Outpost {}
}
