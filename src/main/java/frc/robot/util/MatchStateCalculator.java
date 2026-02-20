package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchStateCalculator {
  public static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static final int[] RED_FRONT_FACE_IDS = new int[] {10, 4};
  public static final int[] BLUE_FRONT_FACE_IDS = new int[] {25, 20};

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public static Translation2d getStaticHub() {
    boolean isRed = isRedAlliance();
    int primaryTagId = isRed ? RED_FRONT_FACE_IDS[0] : BLUE_FRONT_FACE_IDS[0];
    int secondaryTagId = isRed ? RED_FRONT_FACE_IDS[1] : BLUE_FRONT_FACE_IDS[1];

    Translation2d pos1 = tagLayout.getTagPose(primaryTagId).get().toPose2d().getTranslation();
    Translation2d pos2 = tagLayout.getTagPose(secondaryTagId).get().toPose2d().getTranslation();

    return pos1.plus(pos2).div(2);
  }

  public static Translation2d getHub() {
    return getStaticHub();
  }

  public static Translation2d getMovingHub(
      Pose2d robotPose,
      double robotVxMetersPerSec,
      double robotVyMetersPerSec,
      double timeOfFlight) {
    Translation2d staticHub = getHub();

    double virtualX = staticHub.getX() - (robotVxMetersPerSec * timeOfFlight);
    double virtualY = staticHub.getY() - (robotVyMetersPerSec * timeOfFlight);

    return new Translation2d(virtualX, virtualY);
  }

  public static boolean isInAllianceZone(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double x = robotPose.getX();

    if (alliance == Alliance.Blue) {
      return x >= 0.0 && x <= 5.8;
    } else {
      return x >= 11.7 && x <= 17.55;
    }
  }

  public static boolean isInOppAllianceZone(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double x = robotPose.getX();
    if (alliance == Alliance.Blue) {
      return x >= 11.7 && x <= 17.55;
    } else {
      return x >= 0.0 && x <= 5.8;
    }
  }

  public static boolean isInNeutralZone(Pose2d robotPose) {
    double x = robotPose.getX();
    return x > 5.8 && x < 11.7;
  }
}
