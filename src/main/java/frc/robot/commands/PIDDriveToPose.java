package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class PIDDriveToPose extends Command {
  private final Drive drive;
  private final Pose2d targetPose;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  // Tolerances
  private static final double POSITION_TOLERANCE_M = 0.03; // 3 cm
  private static final double ANGLE_TOLERANCE_RAD = Math.toRadians(2.0);

  // At-target debounce
  private int consecutiveAtTarget = 0;
  private static final int DEBOUNCE_CYCLES = 5; // 100ms at 50Hz

  // Minimum travel guard (prevents instant termination from noise)
  private Pose2d startPose;
  private static final double MIN_TRAVEL_M = 0.04;

  public PIDDriveToPose(Drive drive, Pose2d targetPose, double maxSpeed, double maxAccel) {
    this.drive = drive;
    this.targetPose = targetPose;

    xController =
        new ProfiledPIDController(
            1, 0.0, 0.1, new TrapezoidProfile.Constraints(maxSpeed, maxAccel));
    yController =
        new ProfiledPIDController(
            1, 0.0, 0.1, new TrapezoidProfile.Constraints(maxSpeed, maxAccel));

    thetaController =
        new ProfiledPIDController(
            2,
            0.0,
            0.15,
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedRadPerSec() * 0.5, drive.getMaxAngularSpeedRadPerSec()));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(POSITION_TOLERANCE_M);
    yController.setTolerance(POSITION_TOLERANCE_M);
    thetaController.setTolerance(ANGLE_TOLERANCE_RAD);

    addRequirements(drive);
  }

  public PIDDriveToPose(Drive drive, Pose2d targetPose) {
    this(drive, targetPose, 1.5, 2.0);
  }

  @Override
  public void initialize() {
    Pose2d current = drive.getPose();
    startPose = current;
    consecutiveAtTarget = 0;

    // Reset controllers to current state so there's no initial jump
    xController.reset(current.getX());
    yController.reset(current.getY());
    thetaController.reset(current.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d current = drive.getPose();

    double xSpeed = xController.calculate(current.getX(), targetPose.getX());
    double ySpeed = yController.calculate(current.getY(), targetPose.getY());
    double omegaSpeed =
        thetaController.calculate(
            current.getRotation().getRadians(), targetPose.getRotation().getRadians());

    xSpeed =
        MathUtil.clamp(
            xSpeed,
            -xController.getConstraints().maxVelocity,
            xController.getConstraints().maxVelocity);
    ySpeed =
        MathUtil.clamp(
            ySpeed,
            -yController.getConstraints().maxVelocity,
            yController.getConstraints().maxVelocity);

    ChassisSpeeds fieldRelative = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);

    // Convert to robot-relative and use closed-loop velocity control
    ChassisSpeeds robotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, current.getRotation());
    drive.runVelocityClosedLoop(robotRelative);

    // Logging
    Logger.recordOutput("PIDDriveToPose/TargetPose", targetPose);
    Logger.recordOutput("PIDDriveToPose/XError", xController.getPositionError());
    Logger.recordOutput("PIDDriveToPose/YError", yController.getPositionError());
    Logger.recordOutput(
        "PIDDriveToPose/ThetaErrorDeg", Math.toDegrees(thetaController.getPositionError()));
    Logger.recordOutput("PIDDriveToPose/AtGoal", allAtGoal());
  }

  private boolean allAtGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  @Override
  public boolean isFinished() {
    // must have moved minimum distance from start
    double traveled = drive.getPose().getTranslation().getDistance(startPose.getTranslation());
    if (traveled < MIN_TRAVEL_M) {
      consecutiveAtTarget = 0;
      return false;
    }

    // Debounced at-target check
    if (allAtGoal()) {
      consecutiveAtTarget++;
    } else {
      consecutiveAtTarget = 0;
    }
    return consecutiveAtTarget >= DEBOUNCE_CYCLES;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
