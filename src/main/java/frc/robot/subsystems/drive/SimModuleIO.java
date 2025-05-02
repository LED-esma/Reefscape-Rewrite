// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class SimModuleIO implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = true;
  private boolean turnClosedLoop = true;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
    // These are only used for simulation
  private static final MomentOfInertia SteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia DriveInertia = KilogramSquareMeters.of(0.025);
  private SwerveModulePosition last = new SwerveModulePosition(0, Rotation2d.kZero);
  private SwerveModulePosition state = new SwerveModulePosition(0, Rotation2d.kZero);

  public SimModuleIO(SwerveModuleConstants constants) {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX, DriveInertia.baseUnitMagnitude(), constants.THROTTLE_RATIO),
            DRIVE_GEARBOX);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_GEARBOX, SteerInertia.baseUnitMagnitude(), constants.STEER_RATIO),
            TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePosition = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveVoltage = driveAppliedVolts;
    inputs.driveCurrent = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.steerConnected = true;
    inputs.encoderConnected = true;
    inputs.absolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.position = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.steerVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.steerVoltage = turnAppliedVolts;
    inputs.steerCurrent = Math.abs(turnSim.getCurrentDrawAmps());

  }
  
  @Override
  public void setState(SwerveModuleState state) {


    driveFFVolts = DRIVE_KS * Math.signum(state.speedMetersPerSecond) + DRIVE_KV * state.speedMetersPerSecond;

    driveController.setSetpoint(state.speedMetersPerSecond);
    
    turnController.setSetpoint(state.angle.getRadians());
  }

  @Override
  public Rotation2d getRotation2d() {
 
    getModuleDelta();

    var twist = new Twist2d(getModuleDelta().distanceMeters, 0, 0);
  
    Rotation2d moduleAngle = Rotation2d.fromDegrees(0);
    moduleAngle = moduleAngle.plus(new Rotation2d(twist.dtheta));
    return moduleAngle;
    
  }

  @Override
  public SwerveModuleState getState(boolean refresh) {
      return new SwerveModuleState(
          driveSim.getAngularVelocityRadPerSec(), new Rotation2d(turnSim.getAngularPositionRad()));
    
  }

  @Override
  public SwerveModulePosition getPosition(boolean refresh) {
    state = new SwerveModulePosition(driveSim.getAngularPositionRad(), new Rotation2d(turnSim.getAngularPositionRad()));
    return state;
  }

  @Override
  public SwerveModulePosition getModuleDelta() {
    var delta = new SwerveModulePosition(getPosition(true).distanceMeters - last.distanceMeters, state.angle);

    last = new SwerveModulePosition(driveController.getSetpoint(), state.angle);

   return delta;
  }

  @Override
  public void setVoltage(Voltage volts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }
}