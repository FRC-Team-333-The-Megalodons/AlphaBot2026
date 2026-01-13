package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Binding;
import java.util.HashMap;
import java.util.Map;

/**
 * Class that handles the switching of contro schemes deterministically, by setting the active button/trigger
 * poll loop accordingly.
 *
 * <p>Can be easily used in the Commands framework.
 */
public class Controls extends SubsystemBase {

  private Map<String, EventLoop> bindingsMap;
  Alert invalidBindAlert;

  public Controls() {
    bindingsMap = new HashMap<>();
    invalidBindAlert = new Alert("Binding not found", AlertType.kWarning);
    invalidBindAlert.set(false);
  }

  /**
   * Create a new control scheme.
   *
   * @return A Binding object, for use with Controller & Trigger objects.
   * 
   * @param controlSchemeName A friendly name for your conrol scheme.
   */
  public Binding createScheme(String controlSchemeName) {
    return new Binding((bind) -> bindingsMap.put(controlSchemeName, bind));
  }

  /**
   * Check if an existing bind has been created by this name.
   *
   * @param bindName The name of the bind.
   * @return A boolean indicating wheter the Binding has been created before.
   */
  public boolean hasScheme(String controlSchemeName) {
    return bindingsMap.containsKey(controlSchemeName);
  }

  /**
   * Sets the active bind being polled on the controller.
   *
   * <p>Can be used in Commands to change bindings based on state.
   */
  public void setActiveScheme(String controlSchemeName) {

    EventLoop newActiveLoop;

    if (bindingsMap.containsKey(controlSchemeName)) {
      newActiveLoop = bindingsMap.get(controlSchemeName);
    } else {
      invalidBindAlert.setText("Binding \'"+ controlSchemeName +"\'' not found - selecting default control scheme instead");
      invalidBindAlert.set(true);
      newActiveLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    }

    CommandScheduler.getInstance().setActiveButtonLoop(newActiveLoop);
  }

  @Override
  public void periodic() {}
}
