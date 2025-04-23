package frc.robot.subsystems.drive;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * The {@code SwerveModule} class represents a single swerve throttle module,
 * including its motors and encoder, and provides methods to control and
 * retrieve the module's state.
 * <p>
 * This class handles the initialization, configuration, and control of
 * the swerve module's throttle and steering motors and the CANcoder encoder.
 * The swerve module's state (including position and velocity) can be set
 * and retrieved using this class.
 */
public class SwerveModuleIO implements ModuleIO {

  protected TalonFX steeringMotor;
  protected TalonFX throttleMotor;
  protected CANcoder CANCoder;

  protected TalonFXConfiguration steeringConfig;

  protected int ModuleNumber;


  protected SwerveModulePosition internalPosition = new SwerveModulePosition();
  protected SwerveModuleState internalState = new SwerveModuleState();
  protected SwerveModuleConstants constants;

  // Inputs from drive motor
  private StatusSignal<Angle> throttlePosition;
  private StatusSignal<AngularVelocity> throttleVelocity;
  private StatusSignal<Current> throttleCurrent;
  private StatusSignal<Voltage> throttleVoltage;

  // Inputs from steering motor
  private StatusSignal<Angle> steeringPosition;
  private StatusSignal<AngularVelocity> angularVelocity;
  private StatusSignal<Current> steeringCurrent;
  private StatusSignal<Voltage> steeringVoltage;

  // Inputs from encoder
  private StatusSignal<Angle> steerPosition;
  private StatusSignal<Angle> steerabsolutePosition;


  //controller signals
  protected VelocityDutyCycle velocity = new VelocityDutyCycle(0);
  protected PositionDutyCycle position = new PositionDutyCycle(0);
  protected VoltageOut tuning = new VoltageOut(0);


  //for dev tuning
  private boolean isTuning = false;
  public boolean isTuningSteering = true;

  
  /**
   * Constructs a new {@code SwerveModule}.
   *
   * @param moduleNumber The module number used to index into configuration arrays.
   * @param config The configuration constants for the swerve module.
   */
  public SwerveModuleIO(int moduleNumber, SwerveModuleConstants config) {
    this.constants = config;
    this.ModuleNumber = moduleNumber;

    // Initialize motors and encoder with their respective CAN IDs from the configuration
    this.steeringMotor = new TalonFX(config.MODULES[ModuleNumber].STEERING_MOTORID());
    this.throttleMotor = new TalonFX(config.MODULES[ModuleNumber].THROTTLE_MOTORID());
    this.CANCoder = new CANcoder(config.MODULES[ModuleNumber].CANCODER_ID());

    //configure the motors 
    motorConfig();

    // Initialize sensor signals for position and velocity
    this.throttlePosition = throttleMotor.getPosition();
    this.throttleVelocity = throttleMotor.getVelocity();
    this.throttleCurrent = throttleMotor.getSupplyCurrent();
    this.throttleVoltage = throttleMotor.getSupplyVoltage();

  //steering motor signals
    this.steeringPosition = steeringMotor.getPosition();
    this.angularVelocity = steeringMotor.getVelocity();
    this.steeringCurrent = steeringMotor.getSupplyCurrent();  
    this.steeringVoltage = steeringMotor.getSupplyVoltage();
    
  //encoder signals
    this.steerPosition = CANCoder.getPosition();
    this.steerabsolutePosition = CANCoder.getAbsolutePosition();
    
    BaseStatusSignal.setUpdateFrequencyForAll(50, 
      throttlePosition, throttleVelocity, throttleCurrent, throttleVoltage,
      steeringPosition, angularVelocity, steeringCurrent, steeringVoltage,
      steerPosition, steerabsolutePosition);

  }

  public SwerveModuleIO() {
    this.throttlePosition = null;
    this.throttleVelocity = null;
    this.steeringPosition = null;
    this.angularVelocity = null;
  }


  //update the inputs from the motors and encoder
  @Override
  public void updateInputs(ModuleIO.ModuleIOInputs inputs){
      
    inputs.encoderConnected = CANCoder.isConnected();
    inputs.absolutePosition = Rotation2d.fromRotations(CANCoder.getAbsolutePosition().getValueAsDouble()).minus(constants.MODULES[ModuleNumber].CANCODER_OFFSET());
    inputs.position = Rotation2d.fromRotations(CANCoder.getPosition().getValueAsDouble());

    inputs.driveConnected = throttleMotor.isConnected();
    inputs.drivePosition = throttlePosition.getValueAsDouble();
    inputs.driveVelocityRadPerSec = throttleVelocity.getValueAsDouble();

    inputs.steerConnected = steeringMotor.isConnected();
    inputs.steerPosition = steeringPosition.getValueAsDouble();
    inputs.steerVelocityRadPerSec = angularVelocity.getValueAsDouble();
  }

  private void motorConfig(){
        // create config objects
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    
    //  ratio
    angleConfig.Feedback.SensorToMechanismRatio = 1;

    // gains
    var angleSlot0 = angleConfig.Slot0;
      angleSlot0.kS = constants.STEER_KS;
      angleSlot0.kV = constants.STEER_KV;
      angleSlot0.kA = constants.STEER_KA;
      angleSlot0.kP = constants.STEER_KP;
      angleSlot0.kI = constants.STEER_KI;
      angleSlot0.kD = constants.STEER_KD;

    driveConfig.Slot0.kP = constants.THROTTLE_KP;
    
    //invert motors
    driveConfig.MotorOutput.Inverted = 
    constants.MODULES[ModuleNumber].INVERTED() 
    ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    // max amperage
    driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    // curent limits
    driveConfig.CurrentLimits.StatorCurrentLimit = 90;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfig.Feedback.SensorToMechanismRatio = constants.THROTTLE_RATIO;

    // max of 10 volts allows
    driveConfig.Voltage.PeakForwardVoltage = 10;
    driveConfig.Voltage.PeakReverseVoltage = -10;

    angleConfig.Feedback.FeedbackRemoteSensorID = CANCoder.getDeviceID();
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

  //encoder sensor reading configs
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
    encoderConfig.MagnetSensor.MagnetOffset = constants.MODULES[ModuleNumber].CANCODER_OFFSET().getRotations();

    // apply configurations
    steeringMotor.getConfigurator().apply(angleConfig);
    throttleMotor.getConfigurator().apply(driveConfig);
    CANCoder.getConfigurator().apply(encoderConfig);

    //zero the encoder
    steeringMotor.setPosition(constants.MODULES[ModuleNumber].CANCODER_OFFSET().getDegrees());
  }
  
  /**
   * Sets the state of the swerve module (angle   and speed).
   *
   * @param state The desired state (angle and speed) for the module.
    steeringMotor.setPosition(constants.MODULES[ModuleNumber].CANCODER_OFFSET().getDegrees());
    **/
  @Override
  public void setState(SwerveModuleState state) {

    //optimize the rotations to minimize change in angle 
    state.optimize(getState(true).angle);

    var wantedRotations = state.angle.getRotations();
    // Set the steering motor to the desired angle
    steeringMotor.setControl(position.withPosition(wantedRotations));

    // Adjust the speed based on the current and desired angles
    var currentAngle = state.angle;
    state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
    double wantedVelocity = state.speedMetersPerSecond;

    // Set the throttle motor to the desired speed
    throttleMotor.setControl(velocity.withVelocity(wantedVelocity * 5));
  }

  /**
   * SwerveModulePosition is an object which contains the module's position and angle.
   *
   * @param refresh If true, updates the position and angle values by refreshing sensor readings.
   * @return The current position of the module.
   */
  @Override
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      // Refresh sensor readings
      throttlePosition.refresh();
      throttleVelocity.refresh();
      steeringPosition.refresh();
      angularVelocity.refresh();
    }

    // Get compensated throttle rotations and angle rotations
    var throttleRotations = BaseStatusSignal.getLatencyCompensatedValue(throttlePosition, throttleVelocity);
    var angleRotations = BaseStatusSignal.getLatencyCompensatedValue(steeringPosition, angularVelocity);

    // Set the internal position with updated values
    internalPosition.distanceMeters = throttleRotations.baseUnitMagnitude();
    internalPosition.angle = Rotation2d.fromRotations(angleRotations.baseUnitMagnitude());

    return internalPosition;
  }


  /**
   * Retrieves the current state (angle and speed) of the swerve module.
   *
   * @param refresh If true, updates the state values by refreshing sensor readings.
   * @return The current state of the swerve module.
   */
  @Override
  public SwerveModuleState getState(boolean refresh) {
    if (refresh) {
      // Refresh readings
      throttleVelocity.refresh();
      steeringPosition.refresh();
    }

    // Update internal state with current sensor values
    internalState.angle = Rotation2d.fromRotations(steeringPosition.getValueAsDouble());
    internalState.speedMetersPerSecond = throttleVelocity.getValueAsDouble();

    return internalState;
  }

  /**
   * Sets the voltage output of the swerve module's motor.
   *
   * @param voltage The desired voltage output.
   * @param type The type of motor to set the voltage to (steering or throttle).
   */
  @Override
  public void setVoltage(Voltage volts){

    if (isTuning) {
      if (isTuningSteering) {
        steeringMotor.setControl(tuning.withOutput(volts));
      } else {
        throttleMotor.setControl(tuning.withOutput(volts));
      }
    } else {
      steeringMotor.setControl(tuning.withOutput(volts));
      throttleMotor.setControl(tuning.withOutput(volts));
    }

  }


   /**
   * Retrieves the change in the module's position since the last update.
   */
  @Override
  public SwerveModulePosition getModuleDelta(){
    return new SwerveModulePosition(getPosition(false).distanceMeters - internalPosition.distanceMeters, internalPosition.angle);
  }
 
  
  /**
   * Retrieves the current angle of the swerve module.
   *
   * @return The current angle of the swerve module.
   */
  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(steeringPosition.getValueAsDouble());
  }

 
  /**
   * Stops both the steering and throttle motors of the swerve module.
   */
  @Override
  public void Stop() {
    // Stop the motors
    steeringMotor.stopMotor();
    throttleMotor.stopMotor();
  }
}