package frc.robot.interfaces;

/**
 * Any subsystem or IO interface which requires Feedback Control, should implement or extend this
 * interface & it's methods.
 */
public interface Automatable {

  /**
   * Lift, extend, or rotate a mechanism to a desired target.
   *
   * @param setpoint the target angle or distance of the mechanism.
   */
  public void moveTo(double setpoint);

  /**
   * Check if the mechanism has reached the target.
   *
   * @return A boolean, indicating if the target has been reached.
   */
  public boolean atTarget();
}
