package frc.robot.subsystems.climb;

import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbIOKraken implements ClimbIO {
  SparkMax climbMotor;
  SparkMaxConfig climbConfig;
  RelativeEncoder climbEncoder;

// sets and defines motor, config, and encoder
  public ClimbIONeo() {
    climbMotor = new SparkMax(FrigginDeviceIdYo, MotorType.kBrushless);
    climbConfig = new SparkMaxConfig();
    climbMotor.configure(
        climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbEncoder = climbMotor.getEncoder();
  }

// doesnt pass the value of the function. it passes Ts function not hte value.... HELP ME
// note to self. this is called a supplier |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         |
//                                         V

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(climbMotor, () -> climbMotor.getBusVoltage(), (value) -> inputs.climbVoltage = value);
    ifOk(climbMotor, () -> climbMotor.getOutputCurrent(), (value) -> inputs.climbVoltage = value);
    ifOk(climbMotor, () -> climbEncoder.getPosition(), (value) -> inputs.climbPosition = value);
    ifOk(climbMotor, () -> climbEncoder.getVelocity(), (value) -> inputs.climbVelocity = value);
    ifOk(climbMotor, () -> climbMotor.getMotorTemperature(), (value) -> inputs.climbTempC = value);

    inputs.climbMotorConnected = sparkStickyFault;

// i deleted it
  }

  @Override
  public void setClimbVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }

}