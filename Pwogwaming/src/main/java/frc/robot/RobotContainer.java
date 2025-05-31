// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.module.ModuleIOKraken;
import frc.robot.subsystems.drive.module.GyroIO;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Drive drive;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

 private void setupSubsystems() {
    if (Constants.getMode() != Mode.REPLAY) { //if mode is real or simulation
      switch (Constants.getRobot()) {
        case COMPBOT: 
          
          frontLeft = new Module(
            new ModuleIOKraken(
                DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                DriveConstants.FRONT_LEFT_CANCODER_CAN_ID,
                DriveConstants.FRONT_LEFT_TURN_ENCODER_OFFSET,
                DriveConstants.frontLeftZeroRotation),
            DriveConstants.FRONT_LEFT_INDEX,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

          frontRight = new Module(
            new ModuleIOKraken(
                DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                DriveConstants.FRONT_RIGHT_CANCODER_CAN_ID,
                DriveConstants.FRONT_RIGHT_TURN_ENCODER_OFFSET,
                DriveConstants.frontRightZeroRotation),
            DriveConstants.FRONT_RIGHT_INDEX,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

          rearLeft = new Module(
            new ModuleIOKraken(
                DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                DriveConstants.REAR_LEFT_CANCODER_CAN_ID,
                DriveConstants.REAR_LEFT_TURN_ENCODER_OFFSET,
                DriveConstants.rearLeftZeroRotation),
            DriveConstants.REAR_LEFT_INDEX,
            DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

          rearRight = new Module(
            new ModuleIOKraken(
                DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                DriveConstants.REAR_RIGHT_CANCODER_CAN_ID,
                DriveConstants.REAR_RIGHT_TURN_ENCODER_OFFSET,
                DriveConstants.rearRightZeroRotation),
            DriveConstants.REAR_RIGHT_INDEX,
            DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);     

          swerveModules = new Module[] {
            frontLeft,
            frontRight,
            rearLeft,
            rearRight
          };

          drive = new Drive(new GyroIO(), swerveModules);

        case SIMBOT:

          simulationVisualizer = new SimulationVisualizer()

          swerveModules = new Module[] {};

// ts gurt. this friggin code makes 4 modules with simulated modules and puts them in swerveModules. 
// which is then used to create a drive. heartwarming!

          for (moduleIndex = 0, moduleIndex < 3, moduleIndex++) {
            simModule = new Module(
            new ModuleIOMaple(
              simulationVisualizer.getInstance().getSwerveModuleSimulation(moduleIndex)),
            );
            swerveModules[moduleIndex] = simModule;
          }

          drive = new Drive(new GyroIO(), swerveModules);

        default:

          break;
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
  }


}
