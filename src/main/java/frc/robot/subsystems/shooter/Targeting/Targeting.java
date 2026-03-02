package frc.robot.subsystems.shooter.Targeting;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Initializable;
import frc.robot.util.MatchStateCalculator;
import frc.robot.util.Zones;

/**
 * TODO - Move:
 * 
 * - All projectile math
 * - All interpolation maps
 * - All targeting logic
 * - All fixed poses & targets
 * 
 * Into this class.
 * 
 * This class depends on the robot pose from the drive.
 * The Turret and Flywheel classes depend on the target angle and RPM from this class respectively.
 * 
 */
public class Targeting extends SubsystemBase implements Initializable {

  private Zones zones;
  private TargetingIO io;

  private AngularVelocity targetSpeed;
  private Angle targetYaw;
  
  // Input from drive
  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<ChassisSpeeds> robotSpeedsSupplier;

  public Targeting(TargetingIO io, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedsSupplier) {
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotSpeedsSupplier = robotSpeedsSupplier;
    
  }

  public Command defaultTargetingBehavior() {
    return run(() -> {
      Pose2d pose = robotPoseSupplier.get();
      ChassisSpeeds speeds = robotSpeedsSupplier.get();
      Translation2d target;

      if(zones.enemy(pose))
        target = io.getNeutralZoneTarget(pose);
      else if(zones.neutral(pose))
        target = io.getAllianceZoneTarget(pose);
      else
        target = io.getHub();
      
      // You now have pose, speeds, and target. Calculate Turret Angle & Flywheel RPM & save to targetAngle & targetSpeed.
    });
  }
  
  @Override
  public void seed() {
    targetSpeed = RotationsPerSecond.zero();
    targetYaw = DriverStation.getAlliance().get() == Alliance.Blue ?
      Rotation2d.kZero.getMeasure() :
      Rotation2d.k180deg.getMeasure();

    setDefaultCommand(defaultTargetingBehavior());
  }

  /**
   * For use as a supplier to the Turret subsystem.
   * 
   * @return the Target angle of the turret
   */
  public Angle getTargetAngle() {
      return targetYaw;
  }

  /**
   * For use as a supplier to the Flywheel subsystem.
   * 
   * @return The target speed of the flywheel.
   */
  public AngularVelocity getTargetSpeed() {
      return targetSpeed;
  }

  public Translation2d lookAheadTesting() {

    // TODO: Fix This - this was originally in RobotContainer
    double lookaheadTime = 0.060;

    Pose2d currentPose = robotPoseSupplier.get();
    var currentVelocity = drive.robotFieldVelocity();

    Pose2d predictedPose =
        currentPose.exp(
            new edu.wpi.first.math.geometry.Twist2d(
                currentVelocity.dx * lookaheadTime,
                currentVelocity.dy * lookaheadTime,
                currentVelocity.dtheta * lookaheadTime));

    double rawDist =
        predictedPose
            .getTranslation()
            .getDistance(MatchStateCalculator.getHub());

    return MatchStateCalculator.getMovingHub(
        predictedPose,
        currentVelocity.dx,
        currentVelocity.dy,
        MatchStateCalculator.getTimeOfFlight(rawDist));
  }
}
