package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface PoseRecorder {
  static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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
