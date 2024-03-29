// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TrajectoryPaths;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.SwerveSubsystem;

// ExitBlueSpkrRightToRightWallCmd will move a relatively long distance,
// towards the note source wall, to clear the starting zone. This 
// Autononmous move trajectory depends on absolute Y coordinates. 
// WARNING: only test this in a large practive area!!!!

public class ExitBlueSpkrRightToRightWallCmd extends SequentialCommandGroup {
  Trajectory exitBlueSpkrRightToRightWall;
  SwerveControllerCommand exitBlueSpkrRightToRightWallCmd;

  /* Constructor */
  public ExitBlueSpkrRightToRightWallCmd(SwerveSubsystem swerveDrive) {
    TrajectoryConfig exitConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                        (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                        .setKinematics(SDC.SWERVE_KINEMATICS);
    exitConfig.setReversed(false);

    exitBlueSpkrRightToRightWall = TrajectoryGenerator.generateTrajectory
                                    (
                                      // Start with rear bumpers up against the subwoofer 
                                      // right side, and left edge of the robot alligned with the 
                                      // sub-woofer front right corner
                                      new Pose2d(Units.inchesToMeters(41.0), 
                                                  Units.inchesToMeters(155.0), 
                                                  Rotation2d.fromDegrees(300.0)),
                                      List.of(new Translation2d(Units.inchesToMeters(42.0), 
                                                                Units.inchesToMeters(154.0)),
                                              new Translation2d(Units.inchesToMeters(100.0), 
                                                                Units.inchesToMeters(75.0)),
                                              new Translation2d(Units.inchesToMeters(160.0), 
                                                                Units.inchesToMeters(18.0))),
                                      new Pose2d(Units.inchesToMeters(161.0),
                                                  Units.inchesToMeters(17.0),
                                                  Rotation2d.fromDegrees(0.0)),
                                      exitConfig
                                    );

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    exitBlueSpkrRightToRightWallCmd = new SwerveControllerCommand(exitBlueSpkrRightToRightWall,
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
                new InstantCommand(()->swerveDrive.resetOdometry(exitBlueSpkrRightToRightWall.getInitialPose())),
                exitBlueSpkrRightToRightWallCmd,
                new InstantCommand(()->swerveDrive.stop())
               );
  }
}