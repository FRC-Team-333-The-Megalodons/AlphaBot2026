package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldLayout {
  static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  static double FIELD_WIDTH = tagLayout.getFieldWidth();
  static double FIELD_LENGTH = tagLayout.getFieldLength();

  private static Translation2d midpoint(Translation2d a, Translation2d b) {
    return a.plus(b).div(2.0);
  }
  // REMOVED `isBlue` check. PathPlanner AutoBuilder handles flipping automatically.
  // ALWAYS provide the Blue Alliance coordinates below.

  public static class Tower {
    public static final Pose2d CLIMBING_POSE = new Pose2d(1.243, 4.642, Rotation2d.kPi);
    public static final Pose2d CLIMBING_STAGING_POSE = new Pose2d(1.243, 5.642, Rotation2d.kPi);
  }

  public static class Hub {
    public static final Pose2d NEAR_FACE = tagLayout.getTagPose(26).get().toPose2d();
    public static final Pose2d FAR_FACE = tagLayout.getTagPose(20).get().toPose2d();
    public static final Pose2d RIGHT_FACE = tagLayout.getTagPose(18).get().toPose2d();
    public static final Pose2d LEFT_FACE = tagLayout.getTagPose(21).get().toPose2d();
  }

  public static class Depot {
    public static final double WIDTH = Units.inchesToMeters(42.0);
    public static final double DEPTH = Units.inchesToMeters(27.0);
    public static final double HEIGHT = Units.inchesToMeters(1.125);
    public static final double DISTANCE_FROM_CENTER_Y = Units.inchesToMeters(75.93);

    public static final Translation2d DEPOT_CENTER =
        new Translation2d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y);
    public static final Translation2d BLUE_DEPOT = new Translation2d(15.5, 1.9);

    public static final Pose2d DEPOT_SCORING_POSITION = new Pose2d(BLUE_DEPOT, Rotation2d.k180deg);
  }

  public static class Outpost {
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);
    public static final double ROBOT_CLEARANCE = Units.inchesToMeters(26);
    public static final Pose2d LEFT_TAG = tagLayout.getTagPose(29).get().toPose2d();

    public static final Pose2d RIGHT_TAG = tagLayout.getTagPose(30).get().toPose2d();
    public static final Translation2d CENTER =
        midpoint(LEFT_TAG.getTranslation(), RIGHT_TAG.getTranslation());
    public static final Pose2d OUTPOST_POSE =
        new Pose2d(CENTER.getX() + ROBOT_CLEARANCE, CENTER.getY(), Rotation2d.fromDegrees(180));
    public static final Pose2d OUTPOST_APPROACH =
        new Pose2d(CENTER.getX() + 1.5, CENTER.getY(), Rotation2d.fromDegrees(180));
  }

  public static class ScoringPosition {
    public static final Pose2d SCORING_POSITION_A =
        new Pose2d(new Translation2d(16.1, 3.6), Rotation2d.fromDegrees(165));
  }
}
