package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Binds;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;

/**
 * Class that handles the switching of contro schemes deterministically, by setting the active button/trigger
 * poll loop accordingly.
 *
 * <p>Can be easily used in the Commands framework.
 */
public class Controls extends SubsystemBase {

  private Map<String, Binds> bindingsMap;
  Alert invalidBindAlert;

  public Controls() {
    bindingsMap = new HashMap<>();
    invalidBindAlert = new Alert("Bind not found", AlertType.kWarning);
    invalidBindAlert.set(false);
  }

  /**
   * Create a new control scheme.
   *
   * @return A Binds object, for use with Controller & Trigger objects.
   * 
   * @param controlSchemeName A friendly name for your control scheme.
   */
  public Binds createScheme(String controlSchemeName) {
    return new Binds((bind) -> bindingsMap.put(controlSchemeName, bind));
  }

  /**
   * Check if an existing bind has been created by this name.
   *
   * @param bindName The name of the bind.
   * @return A boolean indicating wheter the Binds has been created before.
   */
  public boolean hasScheme(String controlSchemeName) {
    return bindingsMap.containsKey(controlSchemeName);
  }

  /**
   * Convert your binds to a Command.
   *  Switch control schemes on-the-fly with a button press.
   * 
   * @param schemeName The name of your Control Scheme.
   * @param cancellationScheme An Optional String, in case the Command gets interrupted and you need to fall back/return to a non-default control scheme.
   * 
   * @return A Command for use with the WPILib Command Framework.
   */
  public Command registerScheme(
    String schemeName,
    Optional<String> cancellationScheme
  ) {
    if(!this.hasScheme(schemeName))
      throw new NoSuchElementException("Error: Control Scheme "+ schemeName +" does not exist. Did you create it beforehand?");
  
    return this.runOnce(() -> this.setActiveScheme(schemeName))
      .andThen(this.idle()
        .handleInterrupt(() -> this.setActiveScheme(cancellationScheme.get()))
        .onlyIf(() -> cancellationScheme.isPresent())
      )
      .ignoringDisable(true)
      .withName(schemeName +"ControlScheme");
  }

  /**
   * Sets the active bind being polled on the controller.
   *
   * <p>Can be used in Commands to change bindings based on state.
   */
  public void setActiveScheme(String controlSchemeName) {

    EventLoop newActiveLoop;

    if (bindingsMap.containsKey(controlSchemeName)) {
      newActiveLoop = bindingsMap.get(controlSchemeName).eventLoop();
    } else {

      // Really programmer shouldn't make calls to nonexistent Control Schemes,
      //    But we really can't afford to error out on the field.
      invalidBindAlert.setText("Bind \'"+ controlSchemeName +"\'' not found - selecting default control scheme instead");
      invalidBindAlert.set(true);
      newActiveLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    }

    CommandScheduler.getInstance().setActiveButtonLoop(newActiveLoop);
  }

  @Override
  public void periodic() {}
}
