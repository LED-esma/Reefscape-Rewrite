package frc.robot.subsystems.drive;

import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.AutoLogOutput;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class DriveControlSystem implements Subsystem {

    // Swerve Config
    private final SwerveModuleConstants mConfig = Constants.SwerveConstants;

    // Swerve DriveTrain
    protected SwerveModuleGroup driveTrain;

    // pose estimator
     private final SwerveDrivePoseEstimator poseEstimator;
    
    // Gyro
    public AHRS mGyro;

    // Gyro offset by 90 degrees
    private Rotation2d gyroOffset = new Rotation2d(Math.PI / 2);

    // Rotation2d reference
    public final AtomicReference<Rotation2d> rotation2dRef = new AtomicReference<>(new Rotation2d());

    public DriveControlSystem(
        ModuleIO... modules
    ) {

        driveTrain = new SwerveModuleGroup(mConfig, modules);
        
        //mGyro = new AHRS(NavXComType.kMXP_SPI);

        //initializes the pose estimator with everything it set to 0
        poseEstimator = new SwerveDrivePoseEstimator(
            mConfig.KINEMATICS,
            getRotation2d(),
            new SwerveModulePosition[]{
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
                new Pose2d()
            );

        ResetHeading();
  
    }


    @Override
    public void periodic() {
        // Update the pose estimator with the current module states and gyro angle
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), driveTrain.getPositions(true));
        driveTrain.periodic();


    }



    // Drive methods
    public void control(ChassisSpeeds request) {
        driveTrain.control(request);
    }

    //reset the drive train heading 
    public synchronized void ResetHeading() {
        if (mGyro != null) {
            mGyro.reset();
        }
    }

    //get the current heading of the drive train
    public synchronized Rotation2d getRotation2d() {
        rotation2dRef.set(mGyro != null ? mGyro.getRotation2d().rotateBy(gyroOffset) : driveTrain.getRotation2d());
        return rotation2dRef.get();
    }

    public synchronized SwerveModulePosition[] getModulePositions() {
        return driveTrain.getPositions(true);
    }


    //gets the current states of the drive train
    public synchronized SwerveModuleState[] getModuleStates() {
        return driveTrain.getStates(true);
    }

    //gets the current speeds of the drive train
    public ChassisSpeeds getCurrentSpeeds() {
        return driveTrain.getCurrentSpeeds();

    }

    //add a vision estimation to pose estimator
    public void addVisionEstimation(
         Pose2d visionPose,
        double timestamp,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
     poseEstimator.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
    }

    //get the current pose of the drive train
    @AutoLogOutput(key = "Drive/Pose2d")
    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPosition();    
    }

 
    //set the pose of the drive train
    public void setPose2d(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), driveTrain.getPositions(true), pose);
    }



}