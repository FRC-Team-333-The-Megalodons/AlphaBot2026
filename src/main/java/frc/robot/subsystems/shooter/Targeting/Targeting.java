package frc.robot.subsystems.shooter.Targeting;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
public class Targeting extends SubsystemBase {

    private Zones zones;
    private TargetingIO io;
    
    // Input from drive
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;

    public Targeting(TargetingIO io, Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedsSupplier) {
      this.io = io;
      this.robotPoseSupplier = robotPoseSupplier;
      this.robotSpeedsSupplier = robotSpeedsSupplier;
    }

    public Command defaultTargeting() {
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
        
        // You now have pose, speeds, and target. Calculate Turret Angle & Flywheel RPM.
      });
    }

    




    /**
     * Output to turret
     * 
     * @return
     */
    public double targetAngle() {
        return 0.0;
    }

    /**
     * Output to flywheel
     * 
     * @return
     */
    public double targetRPM() {
        return 0.0;
    }
}
