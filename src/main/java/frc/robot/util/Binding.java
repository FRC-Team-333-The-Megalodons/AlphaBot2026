package frc.robot.util;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import java.util.function.Consumer;

/**
 * Wrapper class to handle building and storing controller binds & triggers in an organized manner.
 * 
 * <p>
 * You can think of this class as a BindingBuilder.
 *  This class implements a fluent interface, allowing binds to be stacked together.
 *  Create the initial Binding using {@code Controls.createScheme()}, and then chain binds together.
 * 
 * <p>
 * Helper methods exist to quickly add in Test & Teleop mode binds. You can stack your custom binds on afterward.
 * 
 * <p>
 * To switch bindings, see {@code Controls.setActiveScheme()}.
 * 
 * <p> Example of a command which builds a control scheme from scratch, sets binds, and sets the active control scheme:
 * 
 * 
 * <pre>
 * {@code
 *   public static Command customBindExampleFromScratch(
 *     String name,
 *     Controls controls,
 *     CommandPS5Controller controller
 *   ) {
 *     return controls.runOnce(
 *       () -> {
 *         if(!controls.hasScheme(name)) {
 *
 *           controls.createScheme(name)
 *             .addTeleopModeBindsTo(controller)  // Add preset teleop binds to avoid code duplication
 *             .addAdditionalBinds((eventLoop) -> {
 *
 *               // Controller binds
 *               controller.L1(eventLoop).onTrue(() -> ShooterCommands.shootingSequence(...));
 * 
 *               // Trigger binds
 *               new Trigger(eventLoop, () -> hopper.isLoaded()).whileTrue(() -> IntakeCommands.stopIntake());
 * 
 *             })
 *             .save();
 *         }
 *
 *         controls.setActiveScheme(name);
 *       }
 *     );
 *   }
 * }
 * </pre>
 **/
public class Binding {
  private EventLoop loop;
  private Consumer<EventLoop> saveFunc;

  public Binding(Consumer<EventLoop> bindFunc) {
    this(bindFunc, false);
  }

  public Binding(Consumer<EventLoop> bindFunc, boolean useDefaultLoop) {

    saveFunc = bindFunc;
    loop = useDefaultLoop ? CommandScheduler.getInstance().getDefaultButtonLoop() : new EventLoop();
  }

  /**
   * Get the EventLoop.
   *
   * @return the EventLoop, for binding with triggers & controllers.
   */
  public EventLoop eventLoop() {
    return loop;
  }

  /**
   * Helper function for creating a minimal, common Teleop mode bind for various Command states.
   *
   * <p>This method is a fluent interface.
   *
   * @param controller The controller to be bound to.
   * @return A Binding object, which can be chained together with other methods.
   */
  public Binding addTeleopModeBindsTo(CommandPS5Controller controller) {
    this.addAdditionalBinds((eventLoop) -> {});

    return this;
  }

  /**
   * Add addtional miscellaneous binds to custom bindings.
   *
   * <p>This can be chained after {@code addTeleopModeBindsTo(CommandPS5Controller controller) }, to
   * add additional functionality outside of the basic teleop binds for various Command states.
   *
   * <p>This method is a fluent interface.
   *
   * @param bindFunc A Consumer where custom Controller and Trigger binding can take place.
   * @return A Binding object, which can be chained together with other methods.
   */
  public Binding addAdditionalBinds(Consumer<EventLoop> bindFunc) {
    bindFunc.accept(this.eventLoop());
    return this;
  }

  /**
   * Helper function to quickly create the DriverStation Test mode bind.
   *
   * <p>This method is a fluent interface.
   *
   * @param controller
   * @return A Binding object, which can be chained together with other methods.
   */
  public Binding addTestModeBindsTo(CommandPS5Controller controller) {
    this.addAdditionalBinds((eventLoop) -> {
      controller.L1(this.eventLoop()).onTrue(Commands.none());
    });

    return this;
  }

  /**
   * Saves the bind you have made, so that it can be set as the active control scheme in the future.
   */
  public void save() {
    saveFunc.accept(this.eventLoop());
  }
}
