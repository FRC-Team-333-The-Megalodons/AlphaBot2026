package frc.robot.interfaces;

/** Any subsystem or IO interface which can be loaded, should implement or extend this interface. */
public interface Loadable {

  /**
   * Check if mechanism is loaded
   *
   * @return A boolean, indicating wheter the mechanism has a game piece or not.
   */
  public boolean isLoaded();
}
