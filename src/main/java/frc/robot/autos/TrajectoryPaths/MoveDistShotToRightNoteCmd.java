// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TrajectoryPaths;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// This command moves the robot from the Distant Shot Position (center) 
// to the Right Note for pickup. The robot must turn -90 degrees (CW) 
// (to 270 deg) while moving.

public class MoveDistShotToRightNoteCmd extends SequentialCommandGroup {
  Trajectory moveDistShotToRightNote = null;
  SwerveControllerCommand moveDistShotToRightNoteCmd = null;

  /* Constructor */
  public MoveDistShotToRightNoteCmd(SwerveSubsystem swerveDrive) {
    TrajectoryConfig moveConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                        (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                        .setKinematics(SDC.SWERVE_KINEMATICS);
    moveConfig.setReversed(false);

    moveDistShotToRightNote = TrajectoryGenerator.generateTrajectory
                                 (
                                  new Pose2d(Units.inchesToMeters(120.0), 
                                              Units.inchesToMeters(57.0), 
                                              Rotation2d.fromDegrees(0.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(121.0), 
                                                            Units.inchesToMeters(56.0)),
                                          new Translation2d(Units.inchesToMeters(130.0), 
                                                            Units.inchesToMeters(25.0)),
                                          new Translation2d(Units.inchesToMeters(139.0), 
                                                            Units.inchesToMeters(1.0))),
                                  new Pose2d(Units.inchesToMeters(140.0),
                                              Units.inchesToMeters(0.0),
                                              Rotation2d.fromDegrees(280.0)),
                                  moveConfig
                                 );

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    moveDistShotToRightNoteCmd = new SwerveControllerCommand(moveDistShotToRightNote,
                                                            swerveDrive::getPose,
                                                            SDC.SWERVE_KINEMATICS,
                                                            new PIDController(AutoC.KP_X_CONTROLLER, 
                                                                              AutoC.KI_X_CONTROLLER,
                                                                              0),
                                                            new PIDController(AutoC.KP_Y_CONTROLLER, 
                                                                              AutoC.KI_Y_CONTROLLER, 
                                                                              0),
                                                            thetaController,
                                                            swerveDrive::setModuleStates,
                                                            swerveDrive);
    addCommands(
                moveDistShotToRightNoteCmd,
                new InstantCommand(()->swerveDrive.stop())
               );
  }
}
