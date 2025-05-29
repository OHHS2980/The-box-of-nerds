package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public record PIDConfig(double P, double I, double D) {}

  public static final double maxSpeedMetersPerSec = 30.0;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(22.75);
  public static final double trackLength = Units.inchesToMeters(22.75);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, trackLength / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, trackLength / 2.0),
        new Translation2d(trackWidth / 2.0, -trackLength / 2.0),
        new Translation2d(-trackWidth / 2.0, trackLength / 2.0),
        new Translation2d(-trackWidth / 2.0, -trackLength / 2.0)
      };

  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.235);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.89 - Math.PI / 4);
  public static final Rotation2d rearLeftZeroRotation = new Rotation2d(0.710);
  public static final Rotation2d rearRightZeroRotation = new Rotation2d(-3.112);

  public static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.003));

  // Device CAN IDs

  public static final int FRONT_LEFT_DRIVING_CAN_ID = 0;
  public static final int FRONT_LEFT_TURNING_CAN_ID = 0;
  public static final int FRONT_LEFT_CANCODER_CAN_ID = 0;
  public static final int FRONT_LEFT_TURN_ENCODER_OFFSET = 0;

  public static final int FRONT_RIGHT_DRIVING_CAN_ID = 0;
  public static final int FRONT_RIGHT_TURNING_CAN_ID = 0;
  public static final int FRONT_RIGHT_CANCODER_CAN_ID = 0;
  public static final int FRONT_RIGHT_TURN_ENCODER_OFFSET = 0;

  public static final int REAR_LEFT_DRIVING_CAN_ID = 0;
  public static final int REAR_LEFT_TURNING_CAN_ID = 0;
  public static final int REAR_LEFT_CANCODER_CAN_ID = 0;
  public static final int REAR_LEFT_TURN_ENCODER_OFFSET = 0;

  public static final int REAR_RIGHT_DRIVING_CAN_ID = 0;
  public static final int REAR_RIGHT_TURNING_CAN_ID = 0;
  public static final int REAR_RIGHT_CANCODER_CAN_ID = 0;
  public static final int REAR_RIGHT_TURN_ENCODER_OFFSET = 0;


  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double driveMotorReduction = 6.75;
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final PIDConfig drivePID = new PIDConfig(0.15, 0.0, 0.0);
  public static final PIDConfig driveSimPID = new PIDConfig(0.082416, 0.0, 0.0);
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.13968;

  //index (guys what is index..)

  public static final int FRONT_LEFT_INDEX = 0
  public static final int FRONT_LEFT_INDEX = 0
  public static final int FRONT_LEFT_INDEX = 0
  public static final int FRONT_LEFT_INDEX = 0

  // Turn encoder configuration
  public static final int FRONT_LEFT_TURN_ENCODER_OFFSET = 0;
  public static final int FRONT_RIGHT_TURN_ENCODER_OFFSET = 0;
  public static final int REAR_LEFT_TURN_ENCODER_OFFSET = 0;
  public static final int REAR_RIGHT_TURN_ENCODER_OFFSET = 0;

  
  // Turn PID configuration
  public static final PIDConfig turnPID = new PIDConfig(20.0, 0.0, 0.0);
  public static final PIDConfig turnSimPID = new PIDConfig(8.0, 0.0, 0.0);
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));
}