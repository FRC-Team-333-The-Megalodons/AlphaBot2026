package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    this.createDefaultScheme();
  }

  /**
   * Add binds to default loop.
   * 
   * <p>
   * When the robot first powers on, the default control scheme is loaded in.
   * 
   * <p>
   * Thus, we must configure the default control scheme to switch to the other schemes, based on active mode.
   * 
   * <p>
   * If the robot disconnects during the match, it also makes sense to swap back to the default loop, as a reset/safe mode.
   */
  private void createDefaultScheme() {
    io.createScheme("default", true).addBinds((eventLoop) -> {
      RobotModeTriggers.autonomous().onFalse(this.runOnce(() -> io.setActiveScheme("teleop")));
      RobotModeTriggers.teleop().onTrue(this.runOnce(() -> io.setActiveScheme("teleop")));
      RobotModeTriggers.test().onTrue(this.runOnce(() -> io.setActiveScheme("test")));
    }).save();
  }

  /**
   * See {@code ControlsIO.createScheme()}.
   * 
   * @param schemeName A name for your new control scheme.
   * @return A {@code Binds} object, to use with your controller.
   */
  public Binds createScheme(String schemeName) {

    // All new schemes should revert to default control scheme if connection is lost.
    return io.createScheme(schemeName).addBinds((eventLoop) -> {
      new Trigger(eventLoop, DriverStation::isDisabled).onTrue(
        Commands.sequence(
          this.runOnce(() -> io.setActiveScheme("default")).ignoringDisable(true),
          Commands.print("Lost connection! Reverting to default control scheme.")
        )
      );
    });
  }

  /**
   * Convert your binds to a Command. Switch control schemes on-the-fly with a button press.
   *
   * @param schemeName The name of your Control Scheme.
   * @param cancellationScheme An Optional String, in case the Command gets interrupted and you need
   *     to fall back/return to a non-default control scheme.
   * @return A Command for use with the WPILib Command Framework.
   */
  public Command useScheme(String schemeName, Optional<String> cancellationScheme) {
    if (!io.hasScheme(schemeName))
      throw new NoSuchElementException(
        "Error: Control Scheme " + schemeName + " does not exist. Did you create it beforehand?");

    return Commands.sequence(
      this.runOnce(() -> io.setActiveScheme(schemeName)),
      this.idle()
    ).finallyDo((interrupted) -> {
      if(cancellationScheme.isPresent())
        io.setActiveScheme(cancellationScheme.get());
    }).withName(schemeName + "ControlScheme");
  }

  public Command useScheme(String schemeName) {
    return this.useScheme(schemeName, Optional.empty());
  }

  public Command defaultScheme() {
    return this.useScheme("default").ignoringDisable(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Controls", inputs);
  }
}
