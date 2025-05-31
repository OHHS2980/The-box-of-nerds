//this is a cursed concoction of different methods of structuring a module's code.
//experimenting,,


package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.MK4cSwerveModuleConstants;
import frc.robot.util.custom.GainConstants;

// Modules are consturcted here

public class Module {


//declaring and defining cousins (ModuleIO) and needed values

private final ModuleIO io;

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;


    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public Module(ModuleIO io, int index, double chassisAngularOffset) {
        this.io = io;
        this.index = index;
       
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Drive/Module" + index, inputs);

    }

    public void periodic(){
       
        updateInputs();
    
    }

    public void stop(){
        io.setTurnVoltage(0);
        io.setDriveVoltage(0);

    }
}