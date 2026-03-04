package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class ShootingCommands {
  static GenericEntry rpmEntry;

  public static Command dashboardRPMControl(Flywheel flywheel) {

    rpmEntry = Shuffleboard
      .getTab("Shooter Tuning")
      .add("Tuning Target RPM", 0.0)
      .withPosition(0, 0)
      .withSize(2, 1)
      .getEntry();

      return flywheel.runEnd(
        () -> {
          double targetRPM = rpmEntry.getDouble(0.0);
          flywheel.spinAt(targetRPM, false);
        },
        flywheel::stop
      );
    
  }
}
