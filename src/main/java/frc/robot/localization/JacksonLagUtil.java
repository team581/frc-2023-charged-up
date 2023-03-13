// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.localization.LimelightHelpers.LimelightResults;

public class JacksonLagUtil {
  public static void causeLag() {
    ObjectMapper mapper =
        new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    System.out.println("JacksonLagUtil: Causing lag spike...");
    double time = Timer.getFPGATimestamp();
    try {
      LimelightResults results = new LimelightResults();
      mapper.readValue(mapper.writeValueAsString(results), LimelightHelpers.LimelightResults.class);
    } catch (JsonProcessingException e) {
      System.err.println("JacksonLagUtil: Failed to cause lag spike, JSON parser error:");
      e.printStackTrace();
    }
    System.out.println(
        "JacksonLagUtil: Lag spike caused in " + (Timer.getFPGATimestamp() - time) + "s");
  }

  private JacksonLagUtil() {}
}
