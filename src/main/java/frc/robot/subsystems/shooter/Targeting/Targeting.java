package frc.robot.subsystems.shooter.Targeting;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private TargetingIO io;
    
    // Input from drive
    private Supplier<Pose2d> robotPoseSupplier;

    public Targeting(TargetingIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;
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
