package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

// Nesting is convenient. Because nothing is actually multithreaded,
//  we can imply that a metric started while another is live is inherently nested.

public class RobotMetrics {
  protected HashMap<String, Metric> metrics;
  protected ArrayList<String> callstack;

  static String prefix = "Metrics/";
  private static final RobotMetrics instance = new RobotMetrics();
  private static final boolean ENABLED = false;

  private RobotMetrics() {
    metrics = new HashMap<>();
    callstack = new ArrayList<>();
  }

  private static String getCallStack() {
    ArrayList<String> callstack = instance.callstack;
    StringBuilder fullName = new StringBuilder();
    for (int i = 0; i < callstack.size(); ++i) {
      if (i > 0) {
        fullName.append("/");
      }
      fullName.append(callstack.get(i));
    }
    return fullName.toString();
  }

  private static String updateCallstack(String name, boolean remove) {
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
    if (!ENABLED) {
      return;
    }
    HashMap<String, Metric> metrics = instance.metrics;

    String name = updateCallstack(_name, false);

    if (!metrics.containsKey(name)) {
      metrics.put(name, new Metric(name));
    }
    Metric metric = metrics.get(name);
    metric.start();
  }

  public static void stop(String _name) {
    if (!ENABLED) {
      return;
    }
    HashMap<String, Metric> metrics = instance.metrics;

    String name = updateCallstack(_name, true);

    if (!metrics.containsKey(name)) {
      return;
    }
    Metric metric = metrics.get(name);
    metric.stop();

    RobotMetrics.recordOutput(name + "_avg", metric.average());
    RobotMetrics.recordOutput(name + "_max", metric.max());
  }

  // Its a little weird to put this here, but we are out of time, and this is fast.
  
  // The sole purpose of this is to throttle the calls to recordOutput, which weve found
  // in metrics in visualvm to be the cause of significant slowness
  static HashMap<String, Long> lastLogged = new HashMap<>();
  static final long ADVKIT_UPDATE_DELAY_MS = 1000;

  private static boolean okToUpdate(String key) {
    long lastUpdateTime = lastLogged.getOrDefault(key, 0l);

    long elapsed = System.currentTimeMillis() - lastUpdateTime;

    return (elapsed > ADVKIT_UPDATE_DELAY_MS);
  }

  public static void recordUpdate(String key) {
    lastLogged.put(key, System.currentTimeMillis());
  }

  public static void recordOutput(String key, double value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, boolean value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, int value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, long value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, String value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, SwerveModuleState[] value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, ChassisSpeeds value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, Pose2d[] value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, Pose2d value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }

  public static void recordOutput(String key, Pose3d[] value) {
    if (!okToUpdate(key)) {
      return;
    }
    Logger./**/ recordOutput(key, value);
    recordUpdate(key);
  }
}

class Metric {
  String name;
  long total, count, max;
  long last_start;

  public Metric(String _name) {
    this(_name, false);
  }

  public Metric(String _name, boolean start) {
    name = _name;
    total = 0;
    count = 0;
    max = 0;
    last_start = -1;
    if (start) {
      start();
    }
  }

  public void start() {
    if (last_start > 0) {
      return;
    }

    last_start = System.currentTimeMillis();
  }

  public void stop() {
    if (last_start < 0) {
      return;
    }

    long now = System.currentTimeMillis();
    long elapsed = now - last_start;

    max = Math.max(max, elapsed);
    total += elapsed;
    count += 1;
  }

  public double average() {
    if (count < 0) {
      return -1;
    }

    return (double) total / (double) count;
  }

  public long max() {
    return max;
  }

  public String name() {
    return name;
  }
}
