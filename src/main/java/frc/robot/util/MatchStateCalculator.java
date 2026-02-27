package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchStateCalculator {
  public static AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static final int[] RED_FRONT_FACE_IDS = new int[] {10, 4};
  public static final int[] BLUE_FRONT_FACE_IDS = new int[] {25, 20};
  public static final InterpolatingDoubleTreeMap distanceToTimeOfFlight =
      new InterpolatingDoubleTreeMap();

  static {
    // Distance to ToF
    distanceToTimeOfFlight.put(1.57, 0.8);
    distanceToTimeOfFlight.put(2.00, 3.7);
    distanceToTimeOfFlight.put(2.50, 3.77);
    distanceToTimeOfFlight.put(3.0, 4.03);
    distanceToTimeOfFlight.put(3.50, 1.1);
    distanceToTimeOfFlight.put(4.0, 1.1);
  }

  public static double getTimeOfFlight(double distanceMeters) {
    return distanceToTimeOfFlight.get(distanceMeters);
  }

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

  // public static Translation2d getMovingHub(
  //     Pose2d robotPose, Twist2d robotVelocity, double timeOfFlight, double velocityScalar) {
  //   Translation2d staticHub = getHub();
  //   Translation2d velocity_translational = new Translation2d(robotVelocity.dx, robotVelocity.dy);

  //   Translation2d toHub = staticHub.minus(robotPose.getTranslation());
  //   double uncompensatedRange = toHub.getNorm();
  //   Rotation2d robotToGoal = toHub.getAngle();

  //   Translation2d target_relative_velocity =
  //       velocity_translational.rotateBy(robotToGoal.unaryMinus());

  //   double effectiveVx = robotVelocity.dx * velocityScalar;
  //   double effectiveVy = robotVelocity.dy * velocityScalar;

  //   double virtualX = staticHub.getX() + (effectiveVx * timeOfFlight);
  //   double virtualY = staticHub.getY() + (effectiveVy * timeOfFlight);

  //   return new Translation2d(virtualX, virtualY);
  // }

  public static double lastTargetYawVelocityRadPerSec = 0;

  public static Translation2d getMovingHub(
      Pose2d robotPose, double robotVx, double robotVy, double timeOfFlight) {
    Translation2d staticHub = getHub();

    Translation2d toHub = staticHub.minus(robotPose.getTranslation());
    double uncompensatedRange = toHub.getNorm();
    Rotation2d robotToGoalAngle = toHub.getAngle();

    Translation2d fieldVelocity = new Translation2d(robotVx, robotVy);
    Translation2d goalRelativeVelocity = fieldVelocity.rotateBy(robotToGoalAngle.unaryMinus());

 
    double radialVelocity = goalRelativeVelocity.getX();
    double tangentialVelocity = goalRelativeVelocity.getY();

    
    lastTargetYawVelocityRadPerSec = -(tangentialVelocity / uncompensatedRange);

   
    double dragConstant = 1.65; 
    double velocityScalar =
        (timeOfFlight <= 0.01)
            ? 1.0
            : (1.0 - Math.exp(-dragConstant * timeOfFlight)) / (dragConstant * timeOfFlight);

    double scaledRadial = radialVelocity * velocityScalar;
    double scaledTangential = tangentialVelocity * velocityScalar;


    double baseShotSpeed = uncompensatedRange / timeOfFlight;
    double effectiveShotSpeed = baseShotSpeed - scaledRadial;
    if (effectiveShotSpeed <= 0.0) effectiveShotSpeed = 0.001; 

    double angularOffsetRad = Math.atan2(-scaledTangential, effectiveShotSpeed);

    double effectiveRange = timeOfFlight * Math.hypot(scaledTangential, effectiveShotSpeed);

    Rotation2d finalHeading = robotToGoalAngle.plus(Rotation2d.fromRadians(angularOffsetRad));
    Translation2d virtualOffset = new Translation2d(effectiveRange, finalHeading);

    return robotPose.getTranslation().plus(virtualOffset);
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
