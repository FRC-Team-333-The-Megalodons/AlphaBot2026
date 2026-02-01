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
  public static boolean isBlue = MatchStateCalculator.isBlueAlliance();

  public static class Hub {
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
    public static final Translation2d RED_DEPOT = new Translation2d(15.5, 1.9);
    public static final Translation2d BLUE_DEPOT = new Translation2d(15.5, 1.9);

    public static final Pose2d DEPOT_SCORING_POSITION =
        new Pose2d(isBlue ? BLUE_DEPOT : RED_DEPOT, isBlue ? Rotation2d.k180deg : Rotation2d.kZero);
  }

  public static class Outpost {}

  public static class ScoringPosition {
    public static final Pose2d SCORING_POSITION_A =
        new Pose2d(new Translation2d(16.1, 3.6), Rotation2d.fromDegrees(165));
  }
}
