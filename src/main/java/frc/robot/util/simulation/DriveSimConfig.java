package frc.robot.util.simulation;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
/**
 * DriveSimConfig
 * All Sim objects are allowed to remain in codebase 
 * when deployed to the RoboRIo
 */
public class DriveSimConfig {
    
    // Encoder Sim
    private final EncoderSim mLeftEncoderSim;
    private final EncoderSim mRightEncoderSim;

    // Gyro Sim
    private final AnalogGyroSim mGyroSim;

    public DriveSimConfig(AnalogGyro gyro, 
                Encoder leftEncoder, Encoder rightEncoder)
    {
        mLeftEncoderSim = new EncoderSim(leftEncoder);
        mRightEncoderSim = new EncoderSim(rightEncoder);
        mGyroSim = new AnalogGyroSim(gyro);
    }



}
