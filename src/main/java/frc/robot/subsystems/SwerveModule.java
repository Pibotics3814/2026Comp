package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
// CTRE phoenix6 signals API has changed; configure CANcoder with CANcoderConfiguration directly.
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** A cleaned-up SwerveModule implementation with fixed imports and syntax. */
public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder absoluteEncoder;
    private final SparkClosedLoopController drivingPIDController;
    private final SparkClosedLoopController turningPIDController;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    // Small local ModuleConstants to allow compilation
    private static final class ModuleConstants {
        static final double kTurningEncoderPositionFactor = 1.0;
        static final double kTurningEncoderVelocityFactor = 1.0;
        static final double kTurningP = 1.0;
        static final double kTurningI = 0.0;
        static final double kTurningD = 0.0;
        static final double kTurningFF = 0.0;
        static final double kDrivingEncoderPositionFactor = 1.0;
        static final double kDrivingEncoderVelocityFactor = 1.0;
        static final double kDrivingP = 1.0;
        static final double kDrivingI = 0.0;
        static final double kDrivingD = 0.0;
        static final double kDrivingFF = 0.0;
    }

    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID) {
    driveMotor = new SparkMax(driveMotorCANID, SparkLowLevel.MotorType.kBrushless);
    steerMotor = new SparkMax(steerMotorCANID, SparkLowLevel.MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderCANID);

        // Get the PID Controllers
    // Get the closed-loop controllers
    drivingPIDController = driveMotor.getClosedLoopController();
    turningPIDController = steerMotor.getClosedLoopController();

        // Get the encoders
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

    // Note: REVLib 2026 uses configuration APIs instead of restoreFactoryDefaults.
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

        // CANcoder Configuration
        CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration());
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
    // Apply magnet sensor configuration (use defaults or set fields on magnetSensorConfiguration as needed)
    cfg.apply(magnetSensorConfiguration);

        // Steering Motor Configuration
        steerMotor.setInverted(false);
        // Closed-loop controller configuration should be done via SparkMax config/ClosedLoopSlot.
        // TODO: configure closed-loop slot PID gains and wrap-around behavior via SparkMax APIs.

        // Drive Motor Configuration
        driveMotor.setInverted(false);
        // TODO: configure driving encoder conversion factors and closed-loop PID via SparkMax APIs.

    // Note: persistent configuration should be applied via SparkMax configure APIs if needed.

        driveEncoder.setPosition(0);
        driveEncoder.setPosition(0);
        // TODO: read the absolute encoder and set steer encoder position appropriately.
        steerEncoder.setPosition(0);
    }

    /** Get the distance in meters. */
    public double getDistance() {
        return driveEncoder.getPosition();
    }

    /** Get the angle. */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }

    /** Set the swerve module state. */
    public void setState(SwerveModuleState state) {
        // Set closed-loop setpoints. Units and control types must match configured ClosedLoopSlot.
        turningPIDController.setSetpoint(state.angle.getDegrees(), SparkBase.ControlType.kPosition);
        drivingPIDController.setSetpoint(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
    }

}