// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.util.PoseRecorder;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements PoseRecorder {
  private final NetworkTableInstance inst;
  private DoubleArraySubscriber poseSub;
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Vision vision;

  // Map: Distance in meters -> {RPM, HoodAngle}
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
    // flyWheel
    rpmMap.put(1.0, 2500.0);
    rpmMap.put(5.0, 4500.0);
    // Distance (meters), Hood Angle (Degrees)
    hoodMap.put(1.0, 15.0);
    hoodMap.put(3.0, 30.0);
    // We will add more
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public double calculateDistance() {
    Translation2d robotCoordinate = getPose().getTranslation();
    Translation2d goalCoordinates = this.blueHub();

    return robotCoordinate.getDistance(goalCoordinates);
  }

  public double calculateAngle() {
    Translation2d robotCoordinate = getPose().getTranslation();

    // Pick based on alliance
    Translation2d goalCoordinates = this.blueHub();

    return robotCoordinate.minus(goalCoordinates).getAngle().getDegrees();
  }
  /** The Full Auto-Aim Command */
  public Command aimAtHub() {
    return this.run(
        () -> {
          Rotation2d tx = vision.getTargetX(0);
          double distance = calculateDistance();

          // 1. Turret: Relative move based on tx
          Rotation2d targetAngle = Rotation2d.fromRadians(inputs.turretPositionRad).plus(tx);
          io.setTurretPosition(targetAngle);

          // 2. Flywheel & Hood: Lookup from Map
          io.setShooterVelocity(rpmMap.get(distance));
          io.setHoodPosition(Rotation2d.fromDegrees(hoodMap.get(distance)));

          Logger.recordOutput("Turret/TargetAngle", targetAngle);
        });
  }
}
