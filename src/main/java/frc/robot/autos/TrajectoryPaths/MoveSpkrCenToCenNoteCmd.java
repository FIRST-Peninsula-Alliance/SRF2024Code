// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TrajectoryPaths;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.*;
import frc.robot.commands.DoNothingCmd;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
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

// This command moves the robot from the center against the speaker sub-woofer
// to the center pre-staged note. Regarding the Y dimension, while the field is
// asymetrical, all Y movement can be considered symetrical by defining the
// center of the rightmost note as Y = 0.0, regardless of the Alliance color.

public class MoveSpkrCenToCenNoteCmd extends SequentialCommandGroup {
  Trajectory moveSpkrCenToCenNote = null;
  Command firstCmd = null;
  SwerveControllerCommand moveSpkrCenToCenNoteCmd = null;

  /* Constructor */
  public MoveSpkrCenToCenNoteCmd(SwerveSubsystem swerveDrive, boolean resetOdometryFlag) {
    TrajectoryConfig moveConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                        (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                        .setKinematics(SDC.SWERVE_KINEMATICS);
    moveConfig.setReversed(false);

    moveSpkrCenToCenNote = TrajectoryGenerator.generateTrajectory
                                 (
                                  // Start at the origin facing the X direction, i.e. away from the
                                  // speaker goal. Position rear bumpers up against the subwoofer 
                                  // front, centered
                                  new Pose2d(Units.inchesToMeters(75.0), 
                                              Units.inchesToMeters(57.0), 
                                              Rotation2d.fromDegrees(0.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(76.0), 
                                                            Units.inchesToMeters(57.0)),
                                          new Translation2d(Units.inchesToMeters(95.0), 
                                                            Units.inchesToMeters(57.0)),
                                          new Translation2d(Units.inchesToMeters(119.0), 
                                                            Units.inchesToMeters(57.0))),
                                  new Pose2d(Units.inchesToMeters(120.0),
                                              Units.inchesToMeters(57.0),
                                              Rotation2d.fromDegrees(0.0)),
                                  moveConfig
                                 );

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    moveSpkrCenToCenNoteCmd = new SwerveControllerCommand(moveSpkrCenToCenNote,
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
    if (resetOdometryFlag) {
      firstCmd = new InstantCommand(()->swerveDrive.resetOdometry(moveSpkrCenToCenNote.getInitialPose()));
    } else {
      firstCmd = new DoNothingCmd();
    }

    addCommands(
                firstCmd,
                moveSpkrCenToCenNoteCmd,
                new InstantCommand(()->swerveDrive.stop())
               );
  }
}
