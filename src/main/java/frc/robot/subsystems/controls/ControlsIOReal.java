package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Binds;
import java.util.HashMap;
import java.util.Map;

public class ControlsIOReal implements ControlsIO {

  private Map<String, Binds> bindingsMap;
  private String activeScheme;
  Alert invalidBindAlert;

  public ControlsIOReal() {
    bindingsMap = new HashMap<>();
    invalidBindAlert = new Alert("Bind not found", AlertType.kWarning);
    invalidBindAlert.set(false);
  }

  public Binds createScheme(String controlSchemeName) {
    return new Binds((bind) -> bindingsMap.put(controlSchemeName, bind));
  }
  
  public boolean hasScheme(String controlSchemeName) {
    return bindingsMap.containsKey(controlSchemeName);
  }

  public void setActiveScheme(String controlSchemeName) {

    EventLoop newActiveLoop;

    if (bindingsMap.containsKey(controlSchemeName)) {
      newActiveLoop = bindingsMap.get(controlSchemeName).eventLoop();
      activeScheme = controlSchemeName;
    } else {

      // Really shouldn't make calls to nonexistent Control Schemes,
      //    But we really can't afford to error out on the field.
      invalidBindAlert.setText("Bind \'"+ controlSchemeName+ "\'' not found - selecting default control scheme instead");
      invalidBindAlert.set(true);
      newActiveLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
      activeScheme = "Unknown/Default";
    }

    CommandScheduler.getInstance().setActiveButtonLoop(newActiveLoop);
  }

  @Override
  public void updateInputs(ControlsIOInputs inputs) {
    inputs.activeControlScheme = activeScheme;
  }
}
