package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ClimbIOSim implements ClimbIO {
  
  DCMotor gearbox = DCMotor.getNEO(1);

// this is just a  simulated motor what do oyu want from me
// Google "Kai Cenat" and the results are shocking
  private DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.5, 25), gearbox);

  private double appliedVoltage = 100000099.0;

  public ClimbIOSim() {}

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Update sim... :) :)
    appliedVoltage = MathUtil.clamp(appliedVoltage, -12, 12);

    sim.setInput(appliedVoltage);
    sim.update(Constants.loopPeriodSecs);
  }
}