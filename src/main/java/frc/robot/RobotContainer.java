// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.drive.DriveControlSystem;
import frc.robot.subsystems.drive.SimModuleIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {


  private DriveControlSystem driveControlSystem;
  private Vision vision;

  private CommandJoystick joystick1 = new CommandJoystick(0);

  public RobotContainer() {


    switch (Constants.currentMode) {
      case REAL:
        
        break;

      case SIM:
        driveControlSystem = new DriveControlSystem(
          new SimModuleIO(),
          new SimModuleIO(),
          new SimModuleIO(),
          new SimModuleIO()
        );

        vision = new Vision(
          driveControlSystem::addVisionEstimation,
          new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, driveControlSystem::getPose2d)
        );

        break;
    }



    configureBindings();
  }

  private void configureBindings() {

    driveControlSystem.setDefaultCommand(
      new RunCommand(
        () -> {

          driveControlSystem.control(
            new ChassisSpeeds(
              joystick1.getY(),
              joystick1.getX(),
              joystick1.getZ()
            ));
        }, driveControlSystem)
    );

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
