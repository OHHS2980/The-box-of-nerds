package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.EqualsUtil;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class SimulationVisualizer {
  private static SimulationVisualizer instance;

  //VisionSystemSim visionSim = new VisionSystemSim("main");
  TargetModel targetModel = new TargetModel(0.5, 0.25);

  @Getter
  private static SwerveDriveSimulation driveSim =
      new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(1, 2, new Rotation2d()));

  @Getter private IntakeSimulation intakeSim;

  

  public static SimulationVisualizer getInstance() {
    if (instance == null) {
      instance = new SimulationVisualizer();
    }
    return instance;
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps =
        new double[Arena2025Reefscape.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * Arena2025Reefscape.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }

  public SimulationVisualizer() {
    if (Constants.getRobot() == RobotType.SIMBOT) {
      // The pose of where the target is on the field.
      // Its rotation determines where "forward" or the target x-axis points.
      // Let's say this target is flat against the far wall center, facing the blue driver stations.
      //Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
      // The given target model at the given pose
      //VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

      // Add this vision target to the vision system simulation to make it visible
      //visionSim.addVisionTargets(visionTarget);
      Arena2025Reefscape.getInstance().addDriveTrainSimulation(driveSim);
      intakeSim =
          IntakeSimulation.InTheFrameIntake(
              "Algae", driveSim, Distance.ofBaseUnits(0.7, Meters), IntakeSide.FRONT, 1);
    }
  }

  public void periodic() {
    if (Constants.getMode() == Mode.SIM) {
      Arena2025Reefscape.getInstance().simulationPeriodic();
      Logger.recordOutput("Real Location", getRealPose());
      Logger.recordOutput(
          "FieldSimulation/Algae",
          Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Algae"));
      Logger.recordOutput(
          "FieldSimulation/Coral",
          Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Coral"));
    }

    // If algae is in intake display it, otherwise don't
    if (simHasGamePiece()) {
      Pose3d[] algaeArray = {
        new Pose3d(SimulationVisualizer.getInstance().getRealPose())
            .plus(
                new Transform3d(
                    new Translation3d().plus(new Translation3d(0.35, 0, 0.5)),
                    new Rotation3d(0, 0, 0)))
      };
      Logger.recordOutput("FieldSimulation/Held Algae", algaeArray);
    } else {
      Pose3d[] emptyArray = {};
      Logger.recordOutput("FieldSimulation/Held Algae", emptyArray);
    }
  }

  public Pose2d getRealPose() {
    return driveSim.getSimulatedDriveTrainPose();
  }

  public void setPositiion(Pose2d pose) {
    driveSim.setSimulationWorldPose(pose);
  }

  public boolean simHasGamePiece() {
    if (Constants.getMode() == Mode.SIM) {
      return intakeSim.getGamePiecesAmount() == 1;
    } else {
      return false;
    }
  }

  public SwerveModuleSimulation getSwerveModuleSimulation(int moduleNum) {
    return driveSim.getModules()[moduleNum];
  }

  public GyroSimulation getGyroSimulation() {
    return driveSim.getGyroSimulation();
  }

  public boolean inPosition(Pose2d pose, double translationMargin, double rotationMargin) {
    return pose.getTranslation().getDistance(getRealPose().getTranslation()) < translationMargin
        && EqualsUtil.epsilonEquals(
            pose.getRotation().minus(getRealPose().getRotation()).getDegrees(), rotationMargin);
  }
}