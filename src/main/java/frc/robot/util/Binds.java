package frc.robot.util;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

/**
 * Wrapper class to handle building and storing controller binds & triggers in an organized manner.
 *
 * <p>You can think of this class as a BindBuilder. This class implements a fluent interface,
 * allowing binds to be stacked together. Create the initial Binding using {@code
 * Controls.createScheme()}, and then chain binds together.
 *
 * <p>Helper methods exist to quickly add in Test & Teleop mode binds. You can stack your custom
 * binds on afterward.
 *
 * <p>To switch bindings, see {@code ControlsIO.setActiveScheme()} and {@code Controls.useScheme}.
 */
public class Binds {
  private EventLoop loop;
  private List<Consumer<EventLoop>> bindFuncs;
  private Consumer<Binds> saveFunc;

  public Binds(Consumer<Binds> saveFunc, boolean useDefaultLoop) {

    this.bindFuncs = new LinkedList<>();
    this.saveFunc = saveFunc;
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
   * Add custom bindings to your control scheme.
   *
   * <p>This method is a fluent interface.
   *
   * @param bindFunc A Consumer where custom Controller and Trigger binding can take place.
   * @return A Binding object, which can be chained together with other methods.
   */
  public Binds addBinds(Consumer<EventLoop> bindFunc) {
    bindFuncs.add(bindFunc);
    return this;
  }

  /**
   * Generates & saves the bind you've made, so it can be set as the active control scheme later on.
   */
  public void save() {

    for (Consumer<EventLoop> bindFunc : bindFuncs)
      bindFunc.accept(this.eventLoop());

    saveFunc.accept(this);
  }
}
