package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Binds;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that handles the switching of control schemes, by modifying the active button/trigger poll loop.
 *
 * <p>Can be easily used in the Commands framework.
 */
public class Controls extends SubsystemBase {

  private final ControlsIO io;
  private final ControlsIOInputsAutoLogged inputs = new ControlsIOInputsAutoLogged();

  public Controls(ControlsIO io) {
    this.io = io;
  }

  /**
   * See {@code ControlsIO.createScheme()}.
   * 
   * @param schemeName A name for your new control scheme.
   * @return A {@code Binds} object, to use with your controller.
   */
  public Binds createScheme(String schemeName) {
    return io.createScheme(schemeName);
  }

  /**
   * Convert your binds to a Command. Switch control schemes on-the-fly with a button press.
   *
   * @param schemeName The name of your Control Scheme.
   * @param cancellationScheme An Optional String, in case the Command gets interrupted and you need
   *     to fall back/return to a non-default control scheme.
   * @return A Command for use with the WPILib Command Framework.
   */
  public Command registerScheme(String schemeName, Optional<String> cancellationScheme) {
    if (!io.hasScheme(schemeName))
      throw new NoSuchElementException(
        "Error: Control Scheme " + schemeName + " does not exist. Did you create it beforehand?");

    return this.runOnce(() -> io.setActiveScheme(schemeName))
      .andThen(this.idle()
        .handleInterrupt(() -> io.setActiveScheme(cancellationScheme.get()))
        .onlyIf(() -> cancellationScheme.isPresent()))
      .ignoringDisable(true)
      .withName(schemeName + "ControlScheme");
  }

  public Command registerScheme(String schemeName) {
    return this.registerScheme(schemeName, Optional.empty());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Controls", inputs);
  }
}
