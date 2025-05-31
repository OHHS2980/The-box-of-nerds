// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.SimulationVisualizer;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

// adds the angle and time robot was at that angle to the queues. gyatt!
// https://en.wikipedia.org/wiki/Kai_Cenat
  public GyroIONavX() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getAngle)
  } 

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

// Puts all your stuff into the Amazing conveyor belt.. turns it all into fun arrays... then Clears the queue so theres no old data
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream()
            .mapToDouble((Double value) -> value)
            .toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();

  }  

}