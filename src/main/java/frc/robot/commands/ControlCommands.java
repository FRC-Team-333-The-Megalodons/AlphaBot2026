package frc.robot.commands;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Controls;
import java.util.function.Consumer;

public class ControlCommands {

  public static Command standardTeleopModeBinds(
    Controls controls,
    CommandPS5Controller controller
  ) {
    return controls.runOnce(
      () -> {
        if (!controls.hasScheme("teleopMode")) {
          controls.createScheme("teleopMode")
            .addTeleopModeBindsTo(controller)
            .save();
        }

        controls.setActiveScheme("teleopMode");
      }
    );
  }

  public static Command standardTestModeBinds(Controls controls, CommandPS5Controller controller) {
    return controls.runOnce(
      () -> {
        if (!controls.hasScheme("testMode")) {

          controls.createScheme("testMode")
            .addTestModeBindsTo(controller)
            .save();
        }

        controls.setActiveScheme("testMode");
      }
    );
  }

  private static Command modifiedTeleopModeBinds(
      String bindName,
      Controls controls,
      CommandPS5Controller controller,
      Consumer<EventLoop> bindFunc
  ) {
    return controls.runOnce(
      () -> {
        if (!controls.hasScheme(bindName)) {

          controls.createScheme(bindName)
            .addTeleopModeBindsTo(controller)
            .addAdditionalBinds(bindFunc)
            .save();
        }

        controls.setActiveScheme(bindName);
      }
    );
  }

  /*
  public static Command customTelopBindExample(
      String bindName,
      Controls controls,
      CommandPS5Controller controller
  ) {
    return modifiedTeleopModeBinds(
      bindName,
      controls,
      controller,
      (eventLoop) -> {
        controller.L1(eventLoop).onTrue(Commands.print("button pressed"));
        new Trigger(eventLoop, () -> true).onTrue(Commands.print("New Bind"));
      }
    );
  }

  public static Command customBindExampleFromScratch(
      String bindName,
      Controls controls,
      CommandPS5Controller controller
  ) {
    return controls.runOnce(
      () -> {
        if (!controls.hasScheme(bindName)) {

          controls.createScheme(bindName)
            .addAdditionalBinds((eventLoop) -> {

            })
            .save();
        }

        controls.setActiveScheme(bindName);
      }
    );
  }
  */
}
