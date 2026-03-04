package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.AutopilotConstants.*;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * @deprecated Use <code>DriveCommands.driveToPose()</code>.
 */
@Deprecated
public class DriveToPose extends Command {
  private final Drive drive;
  private final APTarget target;

  private final ProfiledPIDController thetaController;

  public DriveToPose(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.target = new APTarget(targetPose);

    addRequirements(drive);

    thetaController =
        new ProfiledPIDController(
            5.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec() * 2.0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    thetaController.reset(drive.getPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();

    APResult result = kAutopilot.calculate(currentPose, currentSpeeds, target);

    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), result.targetAngle().getRadians());

    ChassisSpeeds commandSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            result.vx(),
            result.vy(),
            AngularVelocity.ofBaseUnits(omega, RadiansPerSecond),
            currentPose.getRotation());

    drive.runVelocity(commandSpeeds);
  }

  @Override
  public boolean isFinished() {
    return kAutopilot.atTarget(drive.getPose(), target);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
