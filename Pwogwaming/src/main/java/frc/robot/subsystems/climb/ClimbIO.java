package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

//creates a bunch of friggin variables yo. i think.
public interface ClimbIO {
  @AutoLog
  class ClimbIOInputs {
    public boolean intakeMotorConnected = false;

    public double climbVoltage = 0.0;
    public double climbCurrent = 0.0;
    public double climbConfigPosition = 0.0;
    public double climbVeloctiy = 0.0;
    public double climbTempC = 0.0;

  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  public default void setClimbVoltage(double voltage) {}

}