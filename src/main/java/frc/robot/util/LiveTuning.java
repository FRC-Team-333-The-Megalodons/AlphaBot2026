package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import java.util.HashMap;
import java.util.Map;

public class LiveTuning {

  private static final Map<String, DoublePublisher> doublePublishers = new HashMap<>();
  private static final Map<String, BooleanPublisher> boolPublishers = new HashMap<>();
  private static final Map<String, StringPublisher> stringPublishers = new HashMap<>();
  private static final Map<String, StructPublisher<Pose2d>> pose2dPublishers = new HashMap<>();

  private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private static final String ROOT = "/LiveTuning/";

  public static void publish(String key, double value) {
    doublePublishers.computeIfAbsent(key, k -> nt.getDoubleTopic(ROOT + k).publish()).set(value);
  }

  public static void publish(String key, boolean value) {
    boolPublishers.computeIfAbsent(key, k -> nt.getBooleanTopic(ROOT + k).publish()).set(value);
  }

  public static void publish(String key, String value) {
    stringPublishers.computeIfAbsent(key, k -> nt.getStringTopic(ROOT + k).publish()).set(value);
  }

  public static void publish(String key, Pose2d value) {
    pose2dPublishers
        .computeIfAbsent(key, k -> nt.getStructTopic(ROOT + k, Pose2d.struct).publish())
        .set(value);
  }
}
