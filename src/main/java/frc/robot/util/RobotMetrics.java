package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

// Nesting is convenient. Because nothing is actually multithreaded,
//  we can imply that a metric started while another is live is inherently nested.

public class RobotMetrics {
  protected HashMap<String, Metric> metrics;
  protected ArrayList<String> callstack;

  static String prefix = "Metrics";
  public static final RobotMetrics instance = new RobotMetrics();

  private RobotMetrics() {
    metrics = new HashMap<>();
    callstack = new ArrayList<>();
  }

  public static String getCallStack() {
    return getCallStack(null);
  }

  public static String getCallStack(String suffix) {
    ArrayList<String> callstack = instance.callstack;
    StringBuilder fullName = new StringBuilder();
    fullName.append(prefix);
    for (int i = 0; i < callstack.size(); ++i) {
      fullName.append("/");
      fullName.append(callstack.get(i));
    }
    if (suffix != null) {
      fullName.append("/");
      fullName.append(suffix);
    }
    return fullName.toString();
  }

  public static String updateCallstack(String name, boolean remove) {
    ArrayList<String> callstack = instance.callstack;
    String output;

    if (remove) {
      // In the case of removal, we need to calc the callstack before.
      output = getCallStack();

      if (callstack.size() > 0 && callstack.get(callstack.size() - 1) == name) {
        callstack.remove(callstack.size() - 1);
      }
    } else {
      // In the case of addition, we need to calc the callstack after.
      if (callstack.size() < 1 || callstack.get(callstack.size() - 1) != name) {
        callstack.add(name);
      }
      output = getCallStack();
    }

    return output;
  }

  public static void start(String _name) {
    HashMap<String, Metric> metrics = instance.metrics;

    String name = updateCallstack(_name, false);

    if (!metrics.containsKey(name)) {
      metrics.put(name, new Metric(name));
    }
    metrics.get(name).start();
  }

  public static void stop(String _name) {
    HashMap<String, Metric> metrics = instance.metrics;

    String name = updateCallstack(_name, true);

    if (!metrics.containsKey(name)) {
      return;
    }
    Metric metric = metrics.get(name);
    metric.stop();

    // Logger.recordOutput(name + "_last", metric.last());
    Logger.recordOutput(name + "_avg", metric.average());
    // Logger.recordOutput(name + "_max", metric.max());
  }

  public static void stat(String _name, long value) {
    HashMap<String, Metric> metrics = instance.metrics;
    String name = getCallStack(_name);
    if (!metrics.containsKey(name)) {
      metrics.put(name, new Metric(name));
    }
    Metric metric = metrics.get(name);
    metric.stat(value);
    Logger.recordOutput(name + "_avg", metric.average());
    Logger.recordOutput(name + "_last", metric.last());
  }
}

class Metric {
  String name;
  long total, count, max, last;
  long last_start;

  public Metric(String _name) {
    name = _name;
    total = 0;
    count = 0;
    max = 0;
    last = 0;
    last_start = -1;
  }

  public void start() {
    if (last_start > 0) {
      return;
    }

    last_start = now();
  }

  private long now() {
    return System.nanoTime() / 1000;
  }

  public void stop() {
    if (last_start < 0) {
      return;
    }

    last = now() - last_start;
    last_start = -1;

    max = Math.max(max, last);
    total += last;
    count += 1;
  }

  public void stat(long value) {
    last = value;
    max = Math.max(max, last);
    total += last;
    count += 1;
  }

  public double average() {
    if (count < 0) {
      return -1;
    }

    return (double) total / (double) count;
  }

  public long last() {
    return last;
  }

  public long max() {
    return max;
  }

  public String name() {
    return name;
  }
}
