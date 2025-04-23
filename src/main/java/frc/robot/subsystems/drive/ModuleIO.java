package frc.robot.subsystems.drive;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public boolean encoderConnected = false;
        public Rotation2d absolutePosition = Rotation2d.kZero;
        public Rotation2d position  = Rotation2d.kZero;

        public boolean driveConnected = false;
        public double drivePosition = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveVoltage = 0.0;
        public double driveCurrent = 0.0;

        public boolean steerConnected = false;
        public double steerPosition = 0.0;
        public double steerVelocityRadPerSec = 0.0;
        public double steerVoltage = 0.0;
        public double steerCurrent = 0.0;
        
    }

    public Rotation2d getRotation2d();

    public SwerveModuleState getState(boolean refresh);

    public default void setState(SwerveModuleState state) {}

    public SwerveModulePosition getPosition(boolean refresh);

    public SwerveModulePosition getModuleDelta();
    
    public void setVoltage(Voltage volts);

    public default void Stop() {}

    public default void updateInputs(ModuleIOInputs inputs) {}

}
