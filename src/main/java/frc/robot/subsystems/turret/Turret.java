package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldLayout;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements FieldLayout {
  private final NetworkTableInstance inst;
  private DoubleArraySubscriber poseSub;
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Vision vision;

  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  public Turret(TurretIO io, Vision vision) {
    inst = NetworkTableInstance.getDefault();
    poseSub = inst.getTable("Drive").getDoubleArrayTopic("Pose").subscribe(new double[] {0, 0, 0});
    this.io = io;
    this.vision = vision;
    setupInterpolation();
  }

  private Pose2d getPose() {
    double[] poseData = poseSub.get();
    return new Pose2d(poseData[0], poseData[1], Rotation2d.fromDegrees(poseData[2]));
  }

  private void setupInterpolation() {
    rpmMap.put(1.0, 2500.0);
    rpmMap.put(5.0, 4500.0);
    hoodMap.put(1.0, 15.0);
    hoodMap.put(3.0, 30.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public double calculateDistance() {
    Translation2d robotCoordinate = getPose().getTranslation();
    Translation2d goalCoordinates = this.getHub();
    return robotCoordinate.getDistance(goalCoordinates);
  }

  /*
   Calculates the turret angle relative to the robot.
   Target relative angle = (Angle to Hub in Relative to Field ) - (Robot Heading)
  */
  public Rotation2d calculateTargetRotation() {
    Translation2d robotCoordinate = getPose().getTranslation();
    Translation2d goalCoordinates = this.getHub();

    // Angle from robot to hub in field relative
    Rotation2d fieldRelativeAngle = goalCoordinates.minus(robotCoordinate).getAngle();

    return fieldRelativeAngle.minus(getPose().getRotation());
  }

  public Command aimAtHub() {
    return this.run(
            () -> {
              double distance = calculateDistance();
              Rotation2d targetAngle = calculateTargetRotation();

              io.setTurretPosition(targetAngle);
              io.setShooterVelocity(rpmMap.get(distance));
              io.setHoodPosition(Rotation2d.fromDegrees(hoodMap.get(distance)));

              Logger.recordOutput("Turret/TargetAngle", targetAngle);
              Logger.recordOutput("Turret/DistanceToHub", distance);
            })
        .finallyDo(interrupted -> io.stop());
  }
}
