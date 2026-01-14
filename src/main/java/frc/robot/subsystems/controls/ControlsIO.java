package frc.robot.subsystems.controls;

import frc.robot.util.Binds;
import org.littletonrobotics.junction.AutoLog;

public interface ControlsIO {

  @AutoLog
  public static class ControlsIOInputs {
    public String activeControlScheme;
  }

  /**
   * Create a new control scheme.
   *
   * @return A {@code Binds} object, for use with Controller & Trigger objects.
   * @param controlSchemeName A friendly name for your control scheme.
   */
  Binds createScheme(String controlSchemeName);

  /**
   * Check if an existing bind has been created by this name.
   *
   * @param bindName The name of the bind.
   * @return A boolean indicating whether the Bind has been created before.
   */
  boolean hasScheme(String controlSchemeName);

  /**
   * Sets the active bind being polled on the controller.
   *
   * <p>Can be used in Commands to change bindings based on state.
   */
  void setActiveScheme(String controlSchemeName);

  public default void updateInputs(ControlsIOInputs inputs) {}
}
