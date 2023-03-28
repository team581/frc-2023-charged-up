// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Paths {
  private static final boolean CACHING_ENABLED = true;

  private static Paths instance;

  public static Paths getInstance() {
    if (instance == null) {
      instance = new Paths();
    }

    return instance;
  }

  private final Map<String, WeakReference<List<PathPlannerTrajectory>>> cache = new HashMap<>();

  private Paths() {}

  public List<PathPlannerTrajectory> getPath(AutoKind auto) {
    if (auto == AutoKind.DO_NOTHING) {
      return List.of();
    }

    return load(auto.pathName, auto.constraints);
  }

  public void clearCache() {
    cache.clear();
  }

  private List<PathPlannerTrajectory> load(String pathName, PathConstraints constraints) {
    if (!CACHING_ENABLED) {
      return PathPlanner.loadPathGroup(pathName, constraints);
    }

    if (cache.containsKey(pathName)) {
      WeakReference<List<PathPlannerTrajectory>> ref = cache.get(pathName);
      if (ref.get() != null) {
        return ref.get();
      }
    }

    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName, constraints);
    cache.put(pathName, new WeakReference<>(path));
    return path;
  }
}
