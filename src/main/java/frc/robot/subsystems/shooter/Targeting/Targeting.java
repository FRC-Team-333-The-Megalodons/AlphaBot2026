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
import frc.robot.util.LiveTuning;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class depends on the robot pose from the drive. The Turret and Flywheel classes depend on
 * the target angle and distance from this class respectively.
 *
 * <p>All distances (targetDistance, augmentedTargetDistance) are measured from the TURRET PIVOT,
 * not the robot center. This matches the origin used inside velocityCompensatedCoordinates(). Tune
 * your FlywheelIO.distanceToRPM and TargetingIOReal.distanceToTOF tables using the
 * "Targeting/TurretDistanceToHub" log key.
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

          // 1. Convert field-relative velocity to robot-relative velocity
          Translation2d fieldVel = new Translation2d(vel.dx, vel.dy);
          Translation2d robotVel = fieldVel.rotateBy(pose.getRotation().unaryMinus());

          // 2. Create a Twist2d representing the exact movement over the lookahead time
          Twist2d lookaheadTwist =
              new Twist2d(
                  robotVel.getX() * LOOKAHEAD_TIME_SEC,
                  robotVel.getY() * LOOKAHEAD_TIME_SEC,
                  vel.dtheta * LOOKAHEAD_TIME_SEC);

          // Added the exponential map to accurately predict the pose along a curved
          // trajectory(before it was linear)
          Pose2d predictedPose = pose.exp(lookaheadTwist);

          // All distances are measured from the turret pivot, not the robot center.
          // This matches the origin used inside velocityCompensatedCoordinates().
          Pose2d predictedTurretPose = io.getTurretPose(predictedPose);

          inputs.targetDistance = predictedTurretPose.getTranslation().getDistance(rawTarget);
          inputs.targetYaw = io.getAngleTo(predictedPose, rawTarget).getDegrees();

          final int MAX_ITERATIONS = 10;
          final double TIME_TOLERANCE_SEC = 0.01; // 10ms tolerance

          // 1. Initial Guess for Time of Flight
          double t_guess = io.getTOFFromDistance(inputs.targetDistance);
          Translation2d velocityCompensatedTarget = rawTarget;

          Pose2d turretPoseAtPrediction = io.getTurretPose(predictedPose);
          Translation2d fieldVelocity = new Translation2d(vel.dx, vel.dy);

          // 2. Dynamic Loop
          for (int i = 0; i < MAX_ITERATIONS; i++) {

            // Shift the target based on the current guessed flight time
            velocityCompensatedTarget =
                io.velocityCompensatedCoordinates(predictedPose, fieldVelocity, t_guess, rawTarget);

            // Recalculate distance from the turret to the new virtual target
            double newDistance =
                io.getDistanceFrom(turretPoseAtPrediction, velocityCompensatedTarget);

            // Look up the new Time of Flight based on the new distance
            double newTof = io.getTOFFromDistance(newDistance);

            // If the difference between our old guess and new flight time is negligible, the math
            // has settled
            if (Math.abs(newTof - t_guess) < TIME_TOLERANCE_SEC) {
              break;
            }

            // Otherwise, update the guess and repeat
            t_guess = newTof;
          }

          // augmentedTargetDistance is measured turret → compensated virtual target.
          inputs.augmentedTargetDistance =
              io.getDistanceFrom(io.getTurretPose(predictedPose), velocityCompensatedTarget);
          inputs.augmentedTargetYaw =
              io.getAngleTo(predictedPose, velocityCompensatedTarget).getDegrees();

          Logger.recordOutput("Targeting/RawTarget", new Pose2d(rawTarget, Rotation2d.kZero));
          Logger.recordOutput(
              "Targeting/CompensatedTarget",
              new Pose2d(velocityCompensatedTarget, Rotation2d.kZero));
          Logger.recordOutput("Targeting/RobotVelocityX", vel.dx);
          Logger.recordOutput("Targeting/RobotVelocityY", vel.dy);
          Logger.recordOutput(
              "Targeting/TargetAngVelRadPerSec", io.getLastTargetAngularVelocityRadPerSec());

          Logger.recordOutput("Targeting/TurretDistanceToHub", inputs.augmentedTargetDistance);
          LiveTuning.publish("Targeting/TurretDistanceToHub", inputs.augmentedTargetDistance);
        });
  }

  public Command simpleTargeting() {
    return run(
        () -> {
          Pose2d pose = robotPoseSupplier.get();
          Twist2d vel = robotVelocitySupplier.get();
          Translation2d hub = io.getHub();

          Pose2d turretPose = io.getTurretPose(pose);
          inputs.targetDistance = io.getDistanceFrom(turretPose, hub);
          inputs.targetYaw = io.getAngleTo(pose, hub).getDegrees();

          Translation2d velocityCompensatedTarget =
              io.velocityCompensatedCoordinates(
                  pose, new Translation2d(vel.dx, vel.dy), inputs.targetDistance, hub);

          inputs.augmentedTargetDistance =
              io.getDistanceFrom(io.getTurretPose(pose), velocityCompensatedTarget);
          inputs.augmentedTargetYaw = io.getAngleTo(pose, velocityCompensatedTarget).getDegrees();

          Logger.recordOutput("Targeting/TurretDistanceToHub", inputs.augmentedTargetDistance);
          SmartDashboard.putNumber("Targeting/TurretDistanceToHub", inputs.augmentedTargetDistance);
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
