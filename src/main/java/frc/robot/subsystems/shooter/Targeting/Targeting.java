package frc.robot.subsystems.shooter.Targeting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Initializable;
import frc.robot.interfaces.Zonable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class depends on the robot pose from the drive. The Turret and Flywheel classes depend on
 * the target angle and distance from this class respectively.
 */
public class Targeting extends SubsystemBase implements Initializable, Zonable {

  private final TargetingIOInputsAutoLogged inputs = new TargetingIOInputsAutoLogged();
  private TargetingIO io;
  private final Field2d targetVisualization = new Field2d();

  // Input from drive
  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<Twist2d> robotVelocitySupplier;

  /**
   * Total system latency compensation in seconds. This accounts for: vision processing (~30ms), CAN
   * bus (~5ms), robot code loop (~20ms), mechanical turret response (~40ms). The pose is predicted
   * forward by this amount before calculating the shot.
   */
  private static final double LOOKAHEAD_TIME_SEC = 0.100;

  public Targeting(
      TargetingIO io, Supplier<Pose2d> robotPoseSupplier, Supplier<Twist2d> robotVelocitySupplier) {
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySupplier = robotVelocitySupplier;
  }

  public Command defaultTargetingBehavior() {
    return run(
        () -> {
          Pose2d pose = robotPoseSupplier.get();
          Twist2d vel = robotVelocitySupplier.get();
          Translation2d rawTarget;

          if (inAllianceZone(pose)) rawTarget = io.getHub();
          else rawTarget = io.getAllianceZoneTarget(pose);

          targetVisualization.setRobotPose(new Pose2d(rawTarget, Rotation2d.kZero));

          // Predict the pose forward by the system latency to compensate for delays
          Pose2d predictedPose =
              new Pose2d(
                  pose.getX() + vel.dx * LOOKAHEAD_TIME_SEC,
                  pose.getY() + vel.dy * LOOKAHEAD_TIME_SEC,
                  pose.getRotation().plus(new Rotation2d(vel.dtheta * LOOKAHEAD_TIME_SEC)));

          inputs.targetDistance = predictedPose.getTranslation().getDistance(rawTarget);
          inputs.targetYaw = io.getAngleTo(predictedPose, rawTarget).getDegrees();

          // 2-iteration TOF refinement with velocity compensation
          // Iteration 1: seed TOF from raw distance
          double tofSeed = io.getTOFFromDistance(inputs.targetDistance);
          Translation2d velocityCompensatedTarget =
              io.velocityCompensatedCoordinates(
                  predictedPose, new Translation2d(vel.dx, vel.dy), tofSeed, rawTarget);

          // Iteration 2: refine TOF using compensated distance
          double refinedTof =
              io.getTOFFromDistance(io.getDistanceFrom(predictedPose, velocityCompensatedTarget));
          velocityCompensatedTarget =
              io.velocityCompensatedCoordinates(
                  predictedPose, new Translation2d(vel.dx, vel.dy), refinedTof, rawTarget);

          inputs.augmentedTargetDistance =
              io.getDistanceFrom(predictedPose, velocityCompensatedTarget);
          inputs.augmentedTargetYaw =
              io.getAngleTo(predictedPose, velocityCompensatedTarget).getDegrees();

          // The angular velocity is computed inside velocityCompensatedCoordinates()
          // and cached. It's written to inputs in updateInputs().

          Logger.recordOutput("Targeting/RawTarget", new Pose2d(rawTarget, Rotation2d.kZero));
          Logger.recordOutput(
              "Targeting/CompensatedTarget",
              new Pose2d(velocityCompensatedTarget, Rotation2d.kZero));
          Logger.recordOutput("Targeting/TOF", refinedTof);
          Logger.recordOutput("Targeting/RobotVelocityX", vel.dx);
          Logger.recordOutput("Targeting/RobotVelocityY", vel.dy);
          Logger.recordOutput(
              "Targeting/TargetAngVelRadPerSec", io.getLastTargetAngularVelocityRadPerSec());
        });
  }

  public Command simpleTargeting() {
    return run(
        () -> {
          Pose2d pose = robotPoseSupplier.get();
          Twist2d vel = robotVelocitySupplier.get();
          Translation2d hub = io.getHub();

          inputs.targetDistance = io.getDistanceFromHub(pose);
          inputs.targetYaw = io.getAngleTo(pose, hub).getDegrees();

          Translation2d velocityCompensatedTarget =
              io.velocityCompensatedCoordinates(
                  pose, new Translation2d(vel.dx, vel.dy), inputs.targetDistance, hub);

          inputs.augmentedTargetDistance = io.getDistanceFrom(pose, velocityCompensatedTarget);
          inputs.augmentedTargetYaw = io.getAngleTo(pose, velocityCompensatedTarget).getDegrees();
        });
  }

  @Override
  public void seed() {
    Pose2d turretPose = io.getTurretPose(robotPoseSupplier.get());
    inputs.targetDistance = io.getDistanceFromHub(turretPose);

    inputs.augmentedTargetDistance = inputs.targetDistance;
    inputs.targetYaw =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Rotation2d.kZero.getDegrees()
            : Rotation2d.k180deg.getDegrees();
    inputs.augmentedTargetYaw = inputs.targetYaw;
    inputs.targetAngularVelocityRadPerSec = 0.0;

    targetVisualization.setRobotPose(new Pose2d(io.getHub(), Rotation2d.kZero));
    SmartDashboard.putData(targetVisualization);
  }

  public Angle getTargetAngle() {
    return Degrees.of(inputs.augmentedTargetYaw);
  }

  public Distance getTargetDistance() {
    return Meters.of(inputs.augmentedTargetDistance);
  }

  public double getTargetAngularVelocityRadPerSec() {
    return inputs.targetAngularVelocityRadPerSec;
  }

  public boolean isInAllianceZone() {
    return inAllianceZone(robotPoseSupplier.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Targeting", inputs);
  }
}