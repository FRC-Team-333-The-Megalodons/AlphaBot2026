package frc.robot.subsystems.tracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks the current match shift and publishes visual timer data to Elastic dashboard. 0:00 – 0:20
 * Autonomous (20s) 0:20 – 0:30 Transition — Both (10s) [both alliances active] 0:30 – 0:55 Shift 1
 * (Loser) (25s) [130–105 remaining] 0:55 – 1:20 Shift 2 (Winner) (25s) [105–80 remaining] 1:20 –
 * 1:45 Shift 3 (Loser) (25s) [80–55 remaining] 1:45 – 2:10 Shift 4 (Winner) (25s) [55–30 remaining]
 * 2:10 – 2:40 Endgame — Both (30s) [30–0 remaining]
 */
public class ShiftTracker extends SubsystemBase {

  private static final double AUTO_DURATION_SEC = 20.0;
  private static final double TELEOP_DURATION_SEC = 140.0;

  private static final double TRANSITION_END = 130.0;
  private static final double SHIFT_1_END = 105.0;
  private static final double SHIFT_2_END = 80.0;
  private static final double SHIFT_3_END = 55.0;
  private static final double SHIFT_4_END = 30.0;

  public enum ShiftPhase {
    DISABLED("DISABLED"),
    AUTONOMOUS("AUTONOMOUS"),
    TRANSITION("TRANSITION"),
    SHIFT_1("SHIFT 1"),
    SHIFT_2("SHIFT 2"),
    SHIFT_3("SHIFT 3"),
    SHIFT_4("SHIFT 4"),
    ENDGAME("ENDGAME");

    public final String label;

    ShiftPhase(String label) {
      this.label = label;
    }
  }

  private ShiftPhase currentPhase = ShiftPhase.DISABLED;
  private double timeLeftInShift = 0.0;
  private boolean isOurShift = false;
  private boolean didWeWinAuto = false;
  private boolean autoWinnerKnown = false;
  private Alliance autoWinner = Alliance.Blue;

  public ShiftTracker() {
    SmartDashboard.putString("Shift", ShiftPhase.DISABLED.label);
    SmartDashboard.putNumber("Time Left in Shift", 0.0);
    SmartDashboard.putBoolean("Is Our Shift", false);
    SmartDashboard.putBoolean("Did We Win Auto", false);
  }

  @Override
  public void periodic() {
    update();
    publish();
  }

  private void update() {
    if (DriverStation.isDisabled()) {
      currentPhase = ShiftPhase.DISABLED;
      timeLeftInShift = 0.0;
      isOurShift = false;
      return;
    }

    Alliance ourAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (DriverStation.isAutonomousEnabled()) {
      currentPhase = ShiftPhase.AUTONOMOUS;
      timeLeftInShift = Math.max(0.0, DriverStation.getMatchTime());
      isOurShift = true;
      return;
    }

    if (DriverStation.isTeleopEnabled()) {
      if (!autoWinnerKnown) {
        resolveAutoWinner();
      }

      double teleopRemaining = Math.max(0.0, DriverStation.getMatchTime());

      if (teleopRemaining > TRANSITION_END) {
        // Transition shift
        currentPhase = ShiftPhase.TRANSITION;
        timeLeftInShift = teleopRemaining - TRANSITION_END;
        isOurShift = true;
      } else if (teleopRemaining > SHIFT_1_END) {
        // Shift 1 — auto LOSER's shift
        currentPhase = ShiftPhase.SHIFT_1;
        timeLeftInShift = teleopRemaining - SHIFT_1_END;
        isOurShift = isLoserAlliance(ourAlliance);
      } else if (teleopRemaining > SHIFT_2_END) {
        // Shift 2 — auto WINNER's shift
        currentPhase = ShiftPhase.SHIFT_2;
        timeLeftInShift = teleopRemaining - SHIFT_2_END;
        isOurShift = isWinnerAlliance(ourAlliance);
      } else if (teleopRemaining > SHIFT_3_END) {
        // Shift 3 — auto LOSER's shift
        currentPhase = ShiftPhase.SHIFT_3;
        timeLeftInShift = teleopRemaining - SHIFT_3_END;
        isOurShift = isLoserAlliance(ourAlliance);
      } else if (teleopRemaining > SHIFT_4_END) {
        // Shift 4 — auto WINNER's shift
        currentPhase = ShiftPhase.SHIFT_4;
        timeLeftInShift = teleopRemaining - SHIFT_4_END;
        isOurShift = isWinnerAlliance(ourAlliance);
      } else {
        // Endgame — both alliances
        currentPhase = ShiftPhase.ENDGAME;
        timeLeftInShift = teleopRemaining;
        isOurShift = true;
      }
    }
  }

  /**
   * Attempts to read who won auto from the FMS game-specific message. The FMS sends "R" or "B" once
   * teleop begins. If not available yet (practice mode, no FMS), defaults to our alliance winning
   * so the dashboard still works.
   */
  private void resolveAutoWinner() {
    String msg = DriverStation.getGameSpecificMessage();
    if (msg != null && !msg.isEmpty()) {
      if (msg.equals("R")) {
        autoWinner = Alliance.Red;
        autoWinnerKnown = true;
      } else if (msg.equals("B")) {
        autoWinner = Alliance.Blue;
        autoWinnerKnown = true;
      }
    }

    // If no FMS data after teleop starts, assume our alliance won so the
    // dashboard still shows something useful in practice/testing.
    if (!autoWinnerKnown && DriverStation.getMatchTime() < TRANSITION_END) {
      autoWinner = DriverStation.getAlliance().orElse(Alliance.Blue);
      autoWinnerKnown = true;
    }

    Alliance ourAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    didWeWinAuto = (autoWinner == ourAlliance);
  }

  private boolean isWinnerAlliance(Alliance ourAlliance) {
    return ourAlliance == autoWinner;
  }

  private boolean isLoserAlliance(Alliance ourAlliance) {
    return ourAlliance != autoWinner;
  }

  private void publish() {
    SmartDashboard.putString("Shift", currentPhase.label);
    SmartDashboard.putNumber("Time Left in Shift", Math.round(timeLeftInShift * 10.0) / 10.0);
    SmartDashboard.putBoolean("Is Our Shift", isOurShift);
    SmartDashboard.putBoolean("Did We Win Auto", didWeWinAuto);

    Logger.recordOutput("ShiftTracker/Phase", currentPhase.label);
    Logger.recordOutput("ShiftTracker/TimeLeftInShift", timeLeftInShift);
    Logger.recordOutput("ShiftTracker/IsOurShift", isOurShift);
    Logger.recordOutput("ShiftTracker/DidWeWinAuto", didWeWinAuto);
    Logger.recordOutput("ShiftTracker/AutoWinner", autoWinner.name());
  }

  public ShiftPhase getCurrentPhase() {
    return currentPhase;
  }

  public double getTimeLeftInShift() {
    return timeLeftInShift;
  }

  public boolean isOurShift() {
    return isOurShift;
  }

  public boolean didWeWinAuto() {
    return didWeWinAuto;
  }

  public boolean isEndgame() {
    return currentPhase == ShiftPhase.ENDGAME;
  }
}
