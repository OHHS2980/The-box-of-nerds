// one button run it press it againn break.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimbIntake extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /** Creates a new Climb Notfunnel. */
  public Climb(ClimbIO io) {
    this.io = io;
  }

// comments for me , a Young blood in need of serious asylum
// log code
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

// move it forward?
  public Command climbin() {
    return Commands.run(
        () -> {
          io.setClimbVoltage(12);
        },
        this);
  }

// move it.................. Backwards
  public Command climbout() {
    return Commands.run(
        () -> {
          io.setClimbVoltage(-10);
        },
        this);
  }

// Halt your moving
  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.setClimbVoltage(0);
        },
        this);
  }

}