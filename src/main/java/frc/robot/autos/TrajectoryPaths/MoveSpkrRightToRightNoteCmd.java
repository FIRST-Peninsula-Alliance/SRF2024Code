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

// This command moves the robot from the Right against the speaker sub-woofer
// to the Right pre-staged note.
// Y origin is defined to be at the center of the right most pre-staged note, 
// regardless of Alliance color. This makes the field symetric RRT Auto
// routines.

public class MoveSpkrRightToRightNoteCmd extends SequentialCommandGroup {
  Trajectory moveSpkrRightToRightNote = null;
  SwerveControllerCommand moveSpkrRightToRightNoteCmd = null;

  /* Constructor */
  public MoveSpkrRightToRightNoteCmd(SwerveSubsystem swerveDrive) {
    TrajectoryConfig moveConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                        (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                        .setKinematics(SDC.SWERVE_KINEMATICS);
    moveConfig.setReversed(false);

    moveSpkrRightToRightNote = TrajectoryGenerator.generateTrajectory
                                 (
                                  // Start with rear bumpers up against subwoofer Right side
                                  new Pose2d(Units.inchesToMeters(41.0), 
                                              Units.inchesToMeters(-8.0), 
                                              Rotation2d.fromDegrees(300.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(42.0), 
                                                            Units.inchesToMeters(-9.0)),
                                          new Translation2d(Units.inchesToMeters(68.0), 
                                                            Units.inchesToMeters(-12.0)),
                                          new Translation2d(Units.inchesToMeters(107.0), 
                                                            Units.inchesToMeters(-15.0))),
                                  new Pose2d(Units.inchesToMeters(108.0),
                                              Units.inchesToMeters(-16.0),
                                              Rotation2d.fromDegrees(0.0)),
                                  moveConfig
                                 );

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    moveSpkrRightToRightNoteCmd = new SwerveControllerCommand(moveSpkrRightToRightNote,
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
                new InstantCommand(()->swerveDrive.resetOdometry(moveSpkrRightToRightNote.getInitialPose())),
                moveSpkrRightToRightNoteCmd,
                new InstantCommand(()->swerveDrive.stop())
               );
  }
}

