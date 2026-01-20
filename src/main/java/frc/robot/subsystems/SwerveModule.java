// /* package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.RelativeEncoder;
// import swervelib.parser.PIDFConfig;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import frc.robot.Constants;

// public class SwerveModule {

//     private final SparkMax driveMotor;
//     private final SparkMax steerMotor;
//     private final CANcoder absoluteEncoder;
//     private final SparkClosedLoopController drivingPIDController;
//     private final SparkClosedLoopController turningPIDController;
//     private final RelativeEncoder driveEncoder;
//     private final RelativeEncoder steerEncoder;

//    // PID in PIDF properties in json per YAGSL

    
//     public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID) {
//     driveMotor = new SparkMax(driveMotorCANID, SparkLowLevel.MotorType.kBrushless);
//     steerMotor = new SparkMax(steerMotorCANID, SparkLowLevel.MotorType.kBrushless);
//         absoluteEncoder = new CANcoder(cancoderCANID);

//         // Get the PID Controllers
//     // Get the closed-loop controllers
//     drivingPIDController = driveMotor.getClosedLoopController();
//     turningPIDController = steerMotor.getClosedLoopController();

//         // Get the encoders
//         driveEncoder = driveMotor.getEncoder();
//         steerEncoder = steerMotor.getEncoder();

//         // CANcoder Configuration
//         CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
//         cfg.apply(new CANcoderConfiguration());
//         MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
//         cfg.refresh(magnetSensorConfiguration);
//     // Apply magnet sensor configuration (use defaults or set fields on magnetSensorConfiguration as needed)
//     cfg.apply(magnetSensorConfiguration);

//     }
//     /** Get the distance in meters. */
//         public double getDistance() {
//         return driveEncoder.getPosition();
//     }

//     /** Get the angle. */
//     public Rotation2d getAngle() {
//         return Rotation2d.fromDegrees(steerEncoder.getPosition());
//     }
//     public SparkClosedLoopController getDrivingPIDController() {
//         return drivingPIDController;
//     }

//     public SparkClosedLoopController getTurningPIDController() {
//         return turningPIDController;
//     }

//     /**
//      * Apply PIDF tuning values from YAGSL's PIDFConfig to the internal SparkMax controllers.
//      * This maps the PIDFConfig fields into REV ClosedLoop and FeedForward configs and applies
//      * them to the device, persisting the parameters. Values used here are placeholders and may
//      * need unit conversion depending on your encoder conversion factors.
//      *
//      * @param drivePID PIDF config for the drive (velocity) controller
//      * @param anglePID PIDF config for the angle (position) controller
//      */
//     public void applyYagslPidf(PIDFConfig drivePID, PIDFConfig anglePID) {
//         // slot 0 by default; adjust if you use different slots
//         frc.robot.YagslSparkTuner.applyPIDFToSpark(driveMotor, drivePID, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
//         frc.robot.YagslSparkTuner.applyPIDFToSpark(steerMotor, anglePID, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
//     }

//     /** Get the distance in meters. */
//     /** Set the swerve module state. */
//     public void setState(SwerveModuleState state) {
//         // Set closed-loop setpoints. Units and control types must match configured ClosedLoopSlot.
//         turningPIDController.setSetpoint(state.angle.getDegrees(), SparkBase.ControlType.kPosition);
//         drivingPIDController.setSetpoint(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
//     }

// } */