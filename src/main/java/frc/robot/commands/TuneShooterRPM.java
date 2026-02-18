package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class TuneShooterRPM extends Command {
  private final Flywheel flywheel;
  private final GenericEntry rpmEntry;

  public TuneShooterRPM(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter Tuning");

    rpmEntry = tab.add("Tuning Target RPM", 0.0).withPosition(0, 0).withSize(2, 1).getEntry();
  }

  @Override
  public void execute() {
    double targetRPM = rpmEntry.getDouble(0.0);

    flywheel.setRPM(targetRPM);
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }
}
