// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.junction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.HashMap;
import java.util.Map;

/**
 * A logger that is almost fully compatible with the AdvantageKit `Logger` class. It uses WPILib's
 * DataLog to log the data to a file, and streams the data using NetworkTables 4.
 */
public class Logger {
  private static final String OUTPUT_LOG_ENTRY_PREFIX = "/RealOutputs/";
  private static final String METADATA_LOG_ENTRY_PREFIX = "/RealMetadata/";

  private static Logger instance;

  public static synchronized Logger getInstance() {
    if (instance == null) {
      instance = new Logger();
    }

    return instance;
  }

  public static synchronized Logger getInstance(String logDir) {
    if (instance == null) {
      instance = new Logger(logDir);
    }

    return instance;
  }

  private final Map<String, Integer> keyToId = new HashMap<>();
  private Map<String, String> allMetadata = new HashMap<>();

  private final Map<Integer, BooleanLogEntry> booleanLogs = new HashMap<>();
  private final Map<Integer, DoubleLogEntry> doubleLogs = new HashMap<>();
  private final Map<Integer, FloatLogEntry> floatLogs = new HashMap<>();
  private final Map<Integer, IntegerLogEntry> integerLogs = new HashMap<>();
  private final Map<Integer, StringLogEntry> stringLogs = new HashMap<>();
  private final Map<Integer, BooleanArrayLogEntry> booleanArrayLogs = new HashMap<>();
  private final Map<Integer, DoubleArrayLogEntry> doubleArrayLogs = new HashMap<>();
  private final Map<Integer, FloatArrayLogEntry> floatArrayLogs = new HashMap<>();
  private final Map<Integer, IntegerArrayLogEntry> integerArrayLogs = new HashMap<>();
  private final Map<Integer, StringArrayLogEntry> stringArrayLogs = new HashMap<>();

  private final Map<Integer, BooleanPublisher> booleanPublishers = new HashMap<>();
  private final Map<Integer, DoublePublisher> doublePublishers = new HashMap<>();
  private final Map<Integer, FloatPublisher> floatPublishers = new HashMap<>();
  private final Map<Integer, IntegerPublisher> integerPublishers = new HashMap<>();
  private final Map<Integer, StringPublisher> stringPublishers = new HashMap<>();
  private final Map<Integer, BooleanArrayPublisher> booleanArrayPublishers = new HashMap<>();
  private final Map<Integer, DoubleArrayPublisher> doubleArrayPublishers = new HashMap<>();
  private final Map<Integer, FloatArrayPublisher> floatArrayPublishers = new HashMap<>();
  private final Map<Integer, IntegerArrayPublisher> integerArrayPublishers = new HashMap<>();
  private final Map<Integer, StringArrayPublisher> stringArrayPublishers = new HashMap<>();

  private DataLog log = DataLogManager.getLog();

  private NetworkTable outputTable;
  private NetworkTable metadataTable;

  private boolean running = false;
  private boolean networkTables = false;

  public Logger() {
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public Logger(String logDir) {
    if (RobotBase.isSimulation()) {
      // Log to project directory in simulation
      DataLogManager.start();
      DataLogManager.logNetworkTables(false);
    } else {
      DataLogManager.start(logDir);
    }
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /** Start the logger without streaming to NetworkTables. */
  public void start() {
    start(false);
  }

  /**
   * Start the logger.
   *
   * @param networkTables Whether to stream the data to NetworkTables (makes your code slower)
   */
  public void start(boolean networkTables) {
    running = true;
    this.networkTables = networkTables;

    if (networkTables) {
      var table = NetworkTableInstance.getDefault().getTable("/AdvantageKit");
      outputTable = table.getSubTable("RealOutputs");
      metadataTable = table.getSubTable("RealMetadata");
    }

    for (Map.Entry<String, String> metadata : allMetadata.entrySet()) {
      String key = metadata.getKey();
      String value = metadata.getValue();

      StringLogEntry entry = new StringLogEntry(log, METADATA_LOG_ENTRY_PREFIX + key);
      entry.append(value);
      entry.finish();

      if (networkTables) {
        StringPublisher publisher = metadataTable.getStringTopic(key).publish();
        publisher.set(value);
      }
    }

    allMetadata = null;
    metadataTable = null;
  }

  public void recordOutput(String key, boolean value) {
    if (running) {
      int id = getId(key);
      booleanLogs
          .computeIfAbsent(id, k -> new BooleanLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(value);
      if (networkTables) {
        booleanPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getBooleanTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(value);
      }
    }
  }

  public void recordOutput(String key, double value) {
    if (running) {
      int id = getId(key);
      doubleLogs
          .computeIfAbsent(id, k -> new DoubleLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(value);
      if (networkTables) {
        doublePublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getDoubleTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(value);
      }
    }
  }

  public void recordOutput(String key, float value) {
    if (running) {
      int id = getId(key);
      floatLogs
          .computeIfAbsent(id, k -> new FloatLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(value);
      if (networkTables) {
        floatPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getFloatTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(value);
      }
    }
  }

  public void recordOutput(String key, int value) {
    if (running) {
      int id = getId(key);
      integerLogs
          .computeIfAbsent(id, k -> new IntegerLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(value);
      if (networkTables) {
        integerPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getIntegerTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(value);
      }
    }
  }

  public void recordOutput(String key, String value) {
    if (running) {
      int id = getId(key);
      stringLogs
          .computeIfAbsent(id, k -> new StringLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(value);
      if (networkTables) {
        stringPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getStringTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(value);
      }
    }
  }

  public void recordOutput(String key, boolean[] values) {
    if (running) {
      int id = getId(key);
      booleanArrayLogs
          .computeIfAbsent(id, k -> new BooleanArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(values);
      if (networkTables) {
        booleanArrayPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getBooleanArrayTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(values);
      }
    }
  }

  public void recordOutput(String key, double[] values) {
    if (running) {
      int id = getId(key);
      doubleArrayLogs
          .computeIfAbsent(id, k -> new DoubleArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(values);
      if (networkTables) {
        doubleArrayPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getDoubleArrayTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(values);
      }
    }
  }

  public void recordOutput(String key, float[] values) {
    if (running) {
      int id = getId(key);
      floatArrayLogs
          .computeIfAbsent(id, k -> new FloatArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(values);
      if (networkTables) {
        floatArrayPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getFloatArrayTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(values);
      }
    }
  }

  public void recordOutput(String key, long[] values) {
    if (running) {
      int id = getId(key);
      integerArrayLogs
          .computeIfAbsent(id, k -> new IntegerArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(values);
      if (networkTables) {
        integerArrayPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getIntegerArrayTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(values);
      }
    }
  }

  public void recordOutput(String key, int[] values) {
    long[] longs = new long[values.length];

    for (int i = 0; i < values.length; i++) {
      longs[i] = values[i];
    }

    recordOutput(key, longs);
  }

  public void recordOutput(String key, String[] value) {
    if (running) {
      int id = getId(key);
      stringArrayLogs
          .computeIfAbsent(id, k -> new StringArrayLogEntry(log, OUTPUT_LOG_ENTRY_PREFIX + key))
          .append(value);

      if (networkTables) {
        stringArrayPublishers
            .computeIfAbsent(
                id,
                k ->
                    outputTable
                        .getStringArrayTopic(key)
                        .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true)))
            .set(value);
      }
    }
  }

  public void recordOutput(String key, Pose2d... value) {
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }
    recordOutput(key, data);
  }

  public void recordOutput(String key, Pose3d... value) {
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }
    recordOutput(key, data);
  }

  public void recordOutput(String key, Trajectory value) {
    recordOutput(
        key, value.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new));
  }

  public void recordOutput(String key, SwerveModuleState... value) {
    double[] data = new double[value.length * 2];
    for (int i = 0; i < value.length; i++) {
      data[i * 2] = value[i].angle.getRadians();
      data[i * 2 + 1] = value[i].speedMetersPerSecond;
    }
    recordOutput(key, data);
  }

  public void recordMetadata(String key, String value) {
    if (!running) {
      allMetadata.put(key, value == null ? "" : value);
    }
  }

  private int getId(String key) {
    return keyToId.computeIfAbsent(key, k -> keyToId.size());
  }
}
