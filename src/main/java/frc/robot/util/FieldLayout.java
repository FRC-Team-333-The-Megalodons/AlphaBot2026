package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface FieldLayout {

  static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final int[] redHubIDs = new int[] {10, 4};
  public static final int[] blueHubIDs = new int[] {26, 20};

  public default Translation2d getHub() {
    Pose2d poseTag1 =
        DriverStation.getAlliance().equals(Alliance.Red)
            ? tagLayout.getTagPose(redHubIDs[0]).get().toPose2d()
            : tagLayout.getTagPose(blueHubIDs[0]).get().toPose2d();

    Pose2d poseTag2 =
        DriverStation.getAlliance().equals(Alliance.Red)
            ? tagLayout.getTagPose(redHubIDs[1]).get().toPose2d()
            : tagLayout.getTagPose(blueHubIDs[1]).get().toPose2d();

    return poseTag1.getTranslation().plus(poseTag2.getTranslation()).div(2);
  }

  public default Translation2d redHub() {

    Pose2d poseA = tagLayout.getTagPose(10).get().toPose2d();

    Pose2d poseB = tagLayout.getTagPose(4).get().toPose2d();
    return poseA.getTranslation().plus(poseB.getTranslation()).div(2);
  }

  public default Translation2d blueHub() {
    Pose2d poseA = tagLayout.getTagPose(26).get().toPose2d();

    Pose2d poseB = tagLayout.getTagPose(20).get().toPose2d();
    return poseA.getTranslation().plus(poseB.getTranslation()).div(2);
  }
}
