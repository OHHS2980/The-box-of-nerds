public class Wrist extends ExampleSubsystem {
  private WristIO io;
  //put inputs
  private SysIdRoutine sysId;
  private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(0, 0));
  private Boolean auto = false;
  private State goalState = new State(0, 0);

  public Wrist(WristIO io) {
    this.io = io;


  /** Creates a new */
  public Wrist() {
    if (Constants.getMode() != Mode.REPLAY) { //if mode is real or simulation
      switch (Constants.getRobot()) {
        case COMPBOT: 
          this.io = new WristIONeo();
          break;
        case SIMBOT:
          this.io = new WristIOSim();
      }
    }
  }


  @Override
  public void periodic() {

  }

  /** Run open loop at specified voltage */
  public void setVoltage(double voltage, boolean safe) {
    auto = false;
    if ((safe && (inputs.armPositionRad > Math.PI && voltage > 0)
        || (inputs.armPositionRad < 0 && voltage < 0))) {
      io.setVoltage(0);
    } else {
      io.setVoltage(voltage);
    }
  }

// log code
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

// i dont know what this will do. mystery
  public Command wristIn() {
    return Commands.run(
        () -> {
          io.setVoltage(12);
        },
        this);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(0);
        },
        this);
  }

}

}