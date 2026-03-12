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
import frc.robot.util.Zones;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class depends on the robot pose from the drive. The Turret and Flywheel classes depend on
 * the target angle and distance from this class respectively.
 */
public class Targeting extends SubsystemBase implements Initializable {

  private final TargetingIOInputsAutoLogged inputs = new TargetingIOInputsAutoLogged();
  private final Zones zones = new Zones();
  private TargetingIO io;
  private final Field2d targetVisualization = new Field2d();

  // Input from drive
  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<Twist2d> robotVelocitySupplier;

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

          double lookaheadTime = 0.060;

          if (zones.alliance(pose)) rawTarget = io.getHub();
          else rawTarget = io.getAllianceZoneTarget(pose);

          targetVisualization.setRobotPose(new Pose2d(rawTarget, Rotation2d.kZero));

          Pose2d predictedPose =
              new Pose2d(
                  pose.getX() + vel.dx * lookaheadTime,
                  pose.getY() + vel.dy * lookaheadTime,
                  pose.getRotation().plus(new Rotation2d(vel.dtheta * lookaheadTime)));

          inputs.targetDistance = predictedPose.getTranslation().getDistance(rawTarget);
          inputs.targetYaw = io.getAngleTo(predictedPose, rawTarget).getDegrees();
          double tof = io.getTOFFromDistance(inputs.targetDistance);

          Translation2d velocityCompensatedTarget =
              io.velocityCompensatedCoordinates(
                  predictedPose, new Translation2d(vel.dx, vel.dy), tof, rawTarget);

          inputs.augmentedTargetDistance =
              io.getDistanceFrom(predictedPose, velocityCompensatedTarget);
          inputs.augmentedTargetYaw =
              io.getAngleTo(predictedPose, velocityCompensatedTarget).getDegrees();

          // Log the raw and compensated targets for AdvantageScope debugging
          Logger.recordOutput("Targeting/RawTarget", new Pose2d(rawTarget, Rotation2d.kZero));
          Logger.recordOutput(
              "Targeting/CompensatedTarget",
              new Pose2d(velocityCompensatedTarget, Rotation2d.kZero));
          Logger.recordOutput("Targeting/TOF", tof);
          Logger.recordOutput("Targeting/RobotVelocityX", vel.dx);
          Logger.recordOutput("Targeting/RobotVelocityY", vel.dy);
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
    inputs.targetDistance = io.getDistanceFromHub(robotPoseSupplier.get());
    inputs.augmentedTargetDistance = inputs.targetDistance;
    inputs.targetYaw =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Rotation2d.kZero.getDegrees()
            : Rotation2d.k180deg.getDegrees();
    inputs.augmentedTargetYaw = inputs.targetYaw;

    targetVisualization.setRobotPose(new Pose2d(io.getHub(), Rotation2d.kZero));
    SmartDashboard.putData(targetVisualization);
  }

  public Angle getTargetAngle() {
    return Degrees.of(inputs.augmentedTargetYaw);
  }

  public Distance getTargetDistance() {
    return Meters.of(inputs.augmentedTargetDistance);
  }

  public boolean isInAllianceZone() {
    return zones.alliance(robotPoseSupplier.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Targeting", inputs);
  }
}
