package frc.robot.interfaces;

/**
 * Any subsystem or IO interface which requires Manual Control for testing, should implement or
 * extend this interface & it's methods.
 */
public interface Teleoperarable {

  /**
   * Move the mechanism manually with the controller.
   *
   * @param input The controller input value, usually from the joystick or analog buttons.
   */
  public void move(double input);
}
