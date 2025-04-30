package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import java.util.Arrays;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.DriveHDC;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIOKraken;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK4cSwerveModuleConstants;
import frc.robot.util.custom.LoggedTunableConstant;


public class Drive extends SubsystemBase {


    private final Module frontLeft, frontRight, rearLeft, rearRight;
    private final Module[] swerveModules;


    frontLeft = new Module(
            new ModuleIOKraken(
                DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                DriveConstants.FRONT_LEFT_CANCODER_CAN_ID,
                DriveConstants.FRONT_LEFT_TURN_ENCODER_OFFSET),
            DriveConstants.FRONT_LEFT_INDEX,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

        frontRight = new Module(
            new ModuleIOKraken(
                DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                DriveConstants.FRONT_RIGHT_CANCODER_CAN_ID,
                DriveConstants.FRONT_RIGHT_TURN_ENCODER_OFFSET),
            DriveConstants.FRONT_RIGHT_INDEX,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

        rearLeft = new Module(
            new ModuleIOKraken(
                DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                DriveConstants.REAR_LEFT_CANCODER_CAN_ID,
                DriveConstants.REAR_LEFT_TURN_ENCODER_OFFSET),
            DriveConstants.REAR_LEFT_INDEX,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

        rearRight = new Module(
            new ModuleIOKraken(
                DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                DriveConstants.REAR_RIGHT_CANCODER_CAN_ID,
                DriveConstants.REAR_RIGHT_TURN_ENCODER_OFFSET),
            DriveConstants.REAR_RIGHT_INDEX,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);     

        swerveModules = new Module[] {
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
        };



}