package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import swervelib.parser.PIDFConfig;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Small utility to map YAGSL PIDFConfig values into REV SparkMax configs and apply them.
 * This is intentionally lightweight: it maps p,i,d -> ClosedLoopConfig.pid and f -> feedForward.kV
 * as a pragmatic placeholder. You may want to refine conversions (units) for your encoders.
 */
public final class YagslSparkTuner {

  private YagslSparkTuner() {}

  /**
   * Apply PIDF values from a YAGSL PIDFConfig to a SparkMax controller.
   * @param spark the SparkMax instance to configure
   * @param pid YAGSL PIDFConfig (p,i,d,f,iz)
   * @param slot target closed-loop slot (use ClosedLoopSlot.kSlot0 by default)
   */
  public static void applyPIDFToSpark(SparkMax spark, PIDFConfig pid, ClosedLoopSlot slot) {
    if (spark == null || pid == null) {
      return;
    }

    try {
      SparkMaxConfig cfg = new SparkMaxConfig();

      // Apply PID gains for the chosen slot
      cfg.closedLoop.pid(pid.p, pid.i, pid.d, slot);

      // Map YAGSL f -> REV kV (volts per velocity) as a placeholder. Adjust if you have a
      // better mapping between units (RPM vs meters/sec etc.).
      cfg.closedLoop.feedForward.kV(pid.f, slot);

      // Optionally set integral zone if provided
      if (pid.iz != 0) {
        cfg.closedLoop.iZone(pid.iz, slot);
      }

      // Apply and persist to the device (use safe reset so we don't wipe unrelated settings)
      spark.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception ex) {
      DriverStation.reportWarning("Failed to apply PIDF to SparkMax: " + ex.getMessage(), true);
    }
  }
}
