package frc.robot.interfaces;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface Localizable {

    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    final Translation2d redHub = tagCoordinates(10).plus(tagCoordinates(4)).div(2);
    final Translation2d blueHub = tagCoordinates(25).plus(tagCoordinates(20)).div(2);
    final Translation2d fieldCenter = new Translation2d(tagLayout.getFieldLength()/2, tagLayout.getFieldWidth()/2);
    final Translation2d xFieldOffset = new Translation2d(Meters.of(4.5), Meters.of(0));
    final Translation2d yFieldOffset = new Translation2d(Meters.of(0), Meters.of(tagLayout.getFieldWidth()/2 - 1));
    final List<Translation2d> redZoneTargets = new ArrayList<>(Arrays.asList(
        fieldCenter.plus(xFieldOffset).plus(yFieldOffset),
        fieldCenter.plus(xFieldOffset).minus(yFieldOffset)
    ));
    final List<Translation2d> blueZoneTargets = new ArrayList<>(Arrays.asList(
        fieldCenter.minus(xFieldOffset).plus(yFieldOffset),
        fieldCenter.minus(xFieldOffset).minus(yFieldOffset)
    ));
    final List<Translation2d> neutralZoneTargets = new ArrayList<>(Arrays.asList(
        fieldCenter.plus(yFieldOffset),
        fieldCenter.minus(yFieldOffset)
    ));

    private static Translation2d tagCoordinates(int tagId) {
        return tagLayout.getTagPose(tagId).get().toPose2d().getTranslation();
    }

    public default Translation2d getHub() {
        return DriverStation.getAlliance().get() == Alliance.Red ? redHub : blueHub;
    }

    public default Translation2d getEnemyHub() {
        return DriverStation.getAlliance().get() == Alliance.Red ? blueHub : redHub;
    }

    public default double getDistanceFromHub(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(getHub());
    }

    public default double getDistanceFromEnemyHub(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(getEnemyHub());
    }

    public default Translation2d getAllianceZoneTarget(Pose2d robotPose) {
        return DriverStation.getAlliance().get() == Alliance.Red ?
            robotPose.getTranslation().nearest(redZoneTargets) :
            robotPose.getTranslation().nearest(blueZoneTargets);
    }

    public default Translation2d getNeutralZoneTarget(Pose2d robotPose) {
        return robotPose.getTranslation().nearest(neutralZoneTargets);
    }

    /*
    public static Rotation2d getAngleToHub(Pose2d robotPose) {

    }
    */
}
