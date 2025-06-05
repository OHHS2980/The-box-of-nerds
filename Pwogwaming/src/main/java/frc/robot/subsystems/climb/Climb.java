// one button run it press it againn break.

package frc.robot.subsystems.climb;
package frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimbIntake extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /** Creates a new Climb Notfunnel. */
  public Climb() {
    if (Constants.getMode() != Mode.REPLAY) { //if mode is real or simulation
      switch (Constants.getRobot()) {
        case COMPBOT: 
          this.io = new ClimbIONeo();
          break;
        case SIMBOT:
          this.io = new ClimbIOSim();
      }
    }
  }

// log code
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

// move it forward
  public Command climbIn() {
    return Commands.run(
        () -> {
          io.setClimbVoltage(12);
        },
        this);
  }

// move it.................. Backwards
  public Command climbOut() {
    return Commands.run(
        () -> {
          io.setClimbVoltage(-10);
        },
        this);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.setClimbVoltage(0);
        },
        this);
  }

}