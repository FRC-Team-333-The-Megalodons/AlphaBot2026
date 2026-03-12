package frc.robot.subsystems.tracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** Tracks the Alliance Shift on the dashboard. */
public class Tracker extends SubsystemBase {

  private static String whoWon = "";
  private static final double autoMatchSeconds = 20.0;
  private static final double teleopMatchSeconds = 140.0;
  private static final double shiftTime = 25.0;

  private static Timer shiftTimer = new Timer();

  // Checks if red won.
  private static BooleanSupplier shiftSelector = () -> whoWon == "R";

  public Tracker() {
    SmartDashboard.putString("Match State", "Disabled");
    SmartDashboard.putBoolean("Blue Auto Won", false);
    SmartDashboard.putBoolean("Blue Shift", false);
    SmartDashboard.putBoolean("Transition", false);
    SmartDashboard.putBoolean("Red Shift", false);
    SmartDashboard.putBoolean("Red Auto Won", false);

    SmartDashboard.putString("Red Shift Time", "N/A");
    SmartDashboard.putNumber("Match Time", 20.0);
    SmartDashboard.putString("Blue Shift Time", "N/A");

    RobotModeTriggers.autonomous()
        .onTrue(
            Commands.parallel(
                runOnce(() -> SmartDashboard.putString("Match State", "Autonomous")),
                Commands.waitSeconds(autoMatchSeconds).deadlineFor(printMatchTime())));
    RobotModeTriggers.teleop()
        .onTrue(
            Commands.parallel(transition(), printMatchTime().asProxy(), signalEndOfShift(true)));

    new Trigger(() -> matchTime() < 130)
        .onTrue(Commands.parallel(autoLoserShift(), signalEndOfShift(false)));

    new Trigger(() -> matchTime() < 105)
        .onTrue(Commands.parallel(autoWinnerShift(), signalEndOfShift(false)));

    new Trigger(() -> matchTime() < 80)
        .onTrue(Commands.parallel(autoLoserShift(), signalEndOfShift(false)));

    new Trigger(() -> matchTime() < 55)
        .onTrue(Commands.parallel(autoWinnerShift(), signalEndOfShift(false)));

    new Trigger(() -> matchTime() < 30).onTrue(endgame());
  }

  private static double matchTime() {
    if (DriverStation.isFMSAttached()) return DriverStation.getMatchTime();
    else if (DriverStation.isAutonomousEnabled())
      return autoMatchSeconds - DriverStation.getMatchTime();
    else if (DriverStation.isTeleopEnabled())
      return teleopMatchSeconds - DriverStation.getMatchTime();
    else return 999.0; // Error
  }

  private static Command shiftCountdown(String key) {
    return Commands.waitSeconds(shiftTime)
        .deadlineFor(
            Commands.startRun(
                () -> shiftTimer.restart(),
                () -> SmartDashboard.putString(key, String.valueOf(shiftTime - shiftTimer.get()))));
  }

  private Command signalEndOfShift(boolean immediately) {
    Command blink =
        Commands.waitSeconds(10.0)
            .deadlineFor(
                Commands.run(
                    () -> {
                      if ((int) matchTime() % 2 == 0) SmartDashboard.putBoolean("Transition", true);
                      else SmartDashboard.putBoolean("Transition", false);
                    }));

    if (immediately) return blink;
    else {
      return Commands.waitSeconds(15.0).andThen(blink);
    }
  }

  public static Command autoLoserShift() {
    return Commands.either(blueShift(), redShift(), shiftSelector);
  }

  public static Command autoWinnerShift() {
    return Commands.either(redShift(), blueShift(), shiftSelector);
  }

  private static Command blueShift() {
    return Commands.runOnce(
            () -> {
              shiftTimer.restart();
              SmartDashboard.putBoolean("Transition", false);
              SmartDashboard.putString("Match State", "Blue Shift");
              SmartDashboard.putBoolean("Blue Shift", true);
              SmartDashboard.putBoolean("Red Shift", false);
              SmartDashboard.putString("Red Shift Time", "N/A");
            })
        .andThen(shiftCountdown("Blue Shift Time"));
  }

  private static Command redShift() {
    return Commands.runOnce(
            () -> {
              shiftTimer.restart();
              SmartDashboard.putBoolean("Transition", false);
              SmartDashboard.putString("Match State", "Red Shift");
              SmartDashboard.putBoolean("Blue Shift", false);
              SmartDashboard.putBoolean("Red Shift", true);
              SmartDashboard.putString("Blue Shift Time", "N/A");
            })
        .andThen(shiftCountdown("Red Shift Time"));
  }

  /**
   * Command to handle <code>DriverStation.getMatchTime()<code>s quirks.
   *
   * @return A Command.
   */
  private static Command printMatchTime() {
    return Commands.run(() -> SmartDashboard.putNumber("Match Time", matchTime()));
  }

  private static Command transition() {
    return Commands.runOnce(
        () -> {
          whoWon = DriverStation.getGameSpecificMessage();

          SmartDashboard.putString("Match State", "Transition Shift");

          if (whoWon == "B") SmartDashboard.putBoolean("Blue Auto Won", true);
          else if (whoWon == "R") SmartDashboard.putBoolean("Red Auto Won", true);
        });
  }

  private static Command endgame() {
    return Commands.runOnce(
        () -> {
          SmartDashboard.putBoolean("Transition", false);
          SmartDashboard.putString("Match State", "Endgame");
          SmartDashboard.putBoolean("Blue Shift", true);
          SmartDashboard.putBoolean("Red Shift", true);
          SmartDashboard.putString("Blue Shift Time", "N/A");
          SmartDashboard.putString("Red Shift Time", "N/A");
        });
  }
}
