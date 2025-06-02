public class Wrist extends ExampleSubsystem {
  private WristIO io;
  //put inputs
  private SysIdRoutine sysId;
  private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(0, 0));
  private Boolean auto = false;
  private State goalState = new State(0, 0);

  public Wrist(WristIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        break;
      case SIM:
        this.io = new WristIOSim();
        break;
      default:
        this.io = new WristIONeo();
        break;
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

  public void setGoal(ArmGoal goal) {
    auto = true;
    this.goal = goal;
  }

  public Rotation2d getArmAngle() {
    return Rotation2d.fromRadians(inputs.armPositionRad);
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(inputs.armPositionRad, goal.getRadians(), 1e-3);
  }
}