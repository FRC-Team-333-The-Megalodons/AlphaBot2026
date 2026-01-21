package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface FieldLayout {
  static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final int[] redHubIDs = new int[] {10, 4};
  // not the right tags for the blue hub tags
  public static final int[] blueHubIDs = new int[] {25, 20};

  public static Translation2d getStaticHub() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    int primaryTagId = isRed ? redHubIDs[0] : blueHubIDs[0];
    int secondaryTagId = isRed ? redHubIDs[1] : blueHubIDs[1];

    Translation2d pos1 = tagLayout.getTagPose(primaryTagId).get().toPose2d().getTranslation();
    Translation2d pos2 = tagLayout.getTagPose(secondaryTagId).get().toPose2d().getTranslation();

    return pos1.plus(pos2).div(2);
  }

  public default Translation2d getHub() {
    return getStaticHub();
  }
}
