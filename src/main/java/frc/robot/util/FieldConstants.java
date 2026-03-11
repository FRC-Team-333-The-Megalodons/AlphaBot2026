package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


public class FieldConstants {


  private static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double FIELD_LENGTH_M = TAG_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_M = TAG_LAYOUT.getFieldWidth();

  
  public static final class FieldLines {

    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH_M / 2.0, FIELD_WIDTH_M / 2.0);

    public static final double CENTER_LINE_X = FIELD_LENGTH_M / 2.0;

    public static final double BLUE_ALLIANCE_WALL_X = 0.0;

    public static final double RED_ALLIANCE_WALL_X = FIELD_LENGTH_M;

    public static final double TOP_WALL_Y = FIELD_WIDTH_M;

    public static final double BOTTOM_WALL_Y = 0.0;

    public static final double MID_FIELD_Y = FIELD_WIDTH_M / 2.0;

    public static final double BLUE_ZONE_LINE_X =
        (TAG_LAYOUT.getTagPose(22).get().getX() + TAG_LAYOUT.getTagPose(23).get().getX()) / 2.0;

   
    public static final double RED_ZONE_LINE_X =
        (TAG_LAYOUT.getTagPose(6).get().getX() + TAG_LAYOUT.getTagPose(7).get().getX()) / 2.0;
  }

  

  
  public static final class Hub {

    // Physical constants — adjust if measured values differ
    // Half-width of one hub face in meters (tags are spread ~0.6m apart on each face)
    private static final double FACE_HALF_WIDTH_M = Units.inchesToMeters(18.0);
    // Clearance from face surface for robot approach poses
    private static final double APPROACH_CLEARANCE_M = Units.inchesToMeters(24.0);


    public static final Translation2d NEAR_FACE_CENTER =
        midpoint(TAG_LAYOUT.getTagPose(25).get().toPose2d().getTranslation(),
                 TAG_LAYOUT.getTagPose(26).get().toPose2d().getTranslation());

    public static final Translation2d FAR_FACE_CENTER =
        midpoint(TAG_LAYOUT.getTagPose(19).get().toPose2d().getTranslation(),
                 TAG_LAYOUT.getTagPose(20).get().toPose2d().getTranslation());

    public static final Translation2d RIGHT_FACE_CENTER =
        midpoint(TAG_LAYOUT.getTagPose(18).get().toPose2d().getTranslation(),
                 TAG_LAYOUT.getTagPose(24).get().toPose2d().getTranslation());

    public static final Translation2d LEFT_FACE_CENTER =
        midpoint(TAG_LAYOUT.getTagPose(21).get().toPose2d().getTranslation(),
                 TAG_LAYOUT.getTagPose(27).get().toPose2d().getTranslation());

    public static final Translation2d CENTER =
        NEAR_FACE_CENTER
            .plus(FAR_FACE_CENTER)
            .plus(RIGHT_FACE_CENTER)
            .plus(LEFT_FACE_CENTER)
            .div(4.0);

   

    public static final Pose2d NEAR_FACE_APPROACH =
        new Pose2d(
            NEAR_FACE_CENTER.minus(
                new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(180))),
            Rotation2d.fromDegrees(0));

    public static final Pose2d FAR_FACE_APPROACH =
        new Pose2d(
            FAR_FACE_CENTER.plus(
                new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(0))),
            Rotation2d.fromDegrees(180));

    public static final Pose2d RIGHT_FACE_APPROACH =
        new Pose2d(
            RIGHT_FACE_CENTER.minus(
                new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(270))),
            Rotation2d.fromDegrees(90));

    public static final Pose2d LEFT_FACE_APPROACH =
        new Pose2d(
            LEFT_FACE_CENTER.plus(
                new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(270))),
            Rotation2d.fromDegrees(-90));

    
    public static final Pose2d NEAR_RIGHT_CORNER = cornerPose(CENTER, 315, APPROACH_CLEARANCE_M);

    public static final Pose2d NEAR_LEFT_CORNER = cornerPose(CENTER, 225, APPROACH_CLEARANCE_M);

    public static final Pose2d FAR_RIGHT_CORNER = cornerPose(CENTER, 45, APPROACH_CLEARANCE_M);

    public static final Pose2d FAR_LEFT_CORNER = cornerPose(CENTER, 135, APPROACH_CLEARANCE_M);
  }

  public static final class Trench {

    private static final double APPROACH_CLEARANCE_M = Units.inchesToMeters(24.0);

    public static final Translation2d LEFT_CENTER =
        midpoint(TAG_LAYOUT.getTagPose(17).get().toPose2d().getTranslation(),
                 TAG_LAYOUT.getTagPose(22).get().toPose2d().getTranslation());

    public static final Translation2d RIGHT_CENTER =
        midpoint(TAG_LAYOUT.getTagPose(23).get().toPose2d().getTranslation(),
                 TAG_LAYOUT.getTagPose(28).get().toPose2d().getTranslation());

   
    public static final Pose2d LEFT_ALLIANCE_SIDE =
        TAG_LAYOUT.getTagPose(17).get().toPose2d();

    public static final Pose2d LEFT_NEUTRAL_SIDE =
        TAG_LAYOUT.getTagPose(22).get().toPose2d();

    public static final Pose2d RIGHT_NEUTRAL_SIDE =
        TAG_LAYOUT.getTagPose(23).get().toPose2d();

    public static final Pose2d RIGHT_ALLIANCE_SIDE =
        TAG_LAYOUT.getTagPose(28).get().toPose2d();

   
    public static final Pose2d LEFT_APPROACH_FROM_ALLIANCE =
        new Pose2d(
            LEFT_CENTER.minus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(0))),
            Rotation2d.fromDegrees(0));

    public static final Pose2d RIGHT_APPROACH_FROM_ALLIANCE =
        new Pose2d(
            RIGHT_CENTER.minus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(0))),
            Rotation2d.fromDegrees(0));

    public static final Pose2d LEFT_APPROACH_FROM_NEUTRAL =
        new Pose2d(
            LEFT_CENTER.plus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(0))),
            Rotation2d.fromDegrees(180));

    public static final Pose2d RIGHT_APPROACH_FROM_NEUTRAL =
        new Pose2d(
            RIGHT_CENTER.plus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(0))),
            Rotation2d.fromDegrees(180));
  }

 
  public static final class Outpost {

    private static final double APPROACH_CLEARANCE_M = Units.inchesToMeters(20.0);

    public static final Pose2d LEFT_TAG =
        TAG_LAYOUT.getTagPose(29).get().toPose2d();

    public static final Pose2d RIGHT_TAG =
        TAG_LAYOUT.getTagPose(30).get().toPose2d();

    public static final Translation2d CENTER =
        midpoint(LEFT_TAG.getTranslation(), RIGHT_TAG.getTranslation());

    public static final Pose2d CENTER_APPROACH =
        new Pose2d(
            CENTER.minus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(180))),
            Rotation2d.fromDegrees(0));

    public static final Pose2d LEFT_APPROACH =
        new Pose2d(
            LEFT_TAG.getTranslation()
                .minus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(180))),
            Rotation2d.fromDegrees(0));

    public static final Pose2d RIGHT_APPROACH =
        new Pose2d(
            RIGHT_TAG.getTranslation()
                .minus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(180))),
            Rotation2d.fromDegrees(0));
  }

  
  public static final class Tower {

    private static final double APPROACH_CLEARANCE_M = Units.inchesToMeters(18.0);

    public static final Pose2d LEFT_TAG =
        TAG_LAYOUT.getTagPose(15).get().toPose2d();

    public static final Pose2d RIGHT_TAG =
        TAG_LAYOUT.getTagPose(16).get().toPose2d();

    public static final Translation2d CENTER =
        midpoint(LEFT_TAG.getTranslation(), RIGHT_TAG.getTranslation());

   
    public static final Pose2d CLIMB_CENTER =
        new Pose2d(CENTER, Rotation2d.fromDegrees(90));

    public static final Pose2d CLIMB_LEFT =
        new Pose2d(
            LEFT_TAG.getTranslation()
                .minus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(270))),
            Rotation2d.fromDegrees(90));

    public static final Pose2d CLIMB_RIGHT =
        new Pose2d(
            RIGHT_TAG.getTranslation()
                .minus(new Translation2d(APPROACH_CLEARANCE_M, Rotation2d.fromDegrees(270))),
            Rotation2d.fromDegrees(90));
  }

  
  private static Translation2d midpoint(Translation2d a, Translation2d b) {
    return a.plus(b).div(2.0);
  }

  private static Pose2d cornerPose(
      Translation2d elementCenter, double angleDeg, double clearance) {
    Rotation2d outwardAngle = Rotation2d.fromDegrees(angleDeg);
    Translation2d offset = new Translation2d(clearance, outwardAngle);
    Translation2d robotPosition = elementCenter.plus(offset);
    Rotation2d robotHeading = outwardAngle.plus(Rotation2d.fromDegrees(180));
    return new Pose2d(robotPosition, robotHeading);
  }

  
  /**
   * Returns the correct Pose2d for the current alliance.  */
  public static Pose2d forAlliance(Pose2d bluePose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FlippingUtil.flipFieldPose(bluePose);
    }
    return bluePose;
  }

  /**
   * Alliance-aware Translation2d overload.
   * Use for target points, not approach poses.
   */
  public static Translation2d forAlliance(Translation2d bluePosition) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FlippingUtil.flipFieldPosition(bluePosition);
    }
    return bluePosition;
  }
}