// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TrajectoryPaths;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.SwerveSubsystem;

// ExitSpkrLeftToLeftWallCmd will move either a short (blue) or long (red) distance,
// depending on Alliance color, to clear the starting zone. This is the
// only Autononmous move trajectory that depends on absolute Y coordinates. 
// If Alliance color cannot be determined, then the move is defaulted to short.
//
// WARNING: This command can cause the robot to move up to 27 feet. Only
// test in a large practive area!!!!
public class ExitSpkrLeftToLeftWallCmd extends SequentialCommandGroup {
  private Optional<Alliance> m_alliance = DriverStation.getAlliance();
  Trajectory exitSpkrLeftToLeftWall;
  Command firstCmd;
  SwerveControllerCommand exitSpkrLeftToLeftWallCmd;

  /** Creates a new ExitSpkrLeftToLeftWallAuto. */
  public ExitSpkrLeftToLeftWallCmd(SwerveSubsystem swerveDrive) {
    TrajectoryConfig exitConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                        (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                        .setKinematics(SDC.SWERVE_KINEMATICS);
    exitConfig.setReversed(false);

    if (m_alliance.isPresent() && (m_alliance.get() == Alliance.Red)) {
        exitSpkrLeftToLeftWall = TrajectoryGenerator.generateTrajectory
                                 (
                                  // Start with rear bumpers up against the subwoofer 
                                  // left side, and right edge of the robot alligned with the 
                                  // sub-woofer front left corner
                                  new Pose2d(Units.inchesToMeters(41.0), 
                                              Units.inchesToMeters(168.0), 
                                              Rotation2d.fromDegrees(60.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(42.0), 
                                                            Units.inchesToMeters(169.0)),
                                          new Translation2d(Units.inchesToMeters(100.0), 
                                                            Units.inchesToMeters(210.0)),
                                          new Translation2d(Units.inchesToMeters(153.0), 
                                                            Units.inchesToMeters(305.0))),
                                  new Pose2d(Units.inchesToMeters(154.0),
                                              Units.inchesToMeters(306.0),
                                              Rotation2d.fromDegrees(0.0)),
                                  exitConfig
                                 );
      } else {    // Blue, or any other but red
        exitSpkrLeftToLeftWall = TrajectoryGenerator.generateTrajectory
                                 (
                                 new Pose2d(Units.inchesToMeters(41.0), 
                                              Units.inchesToMeters(284.0), 
                                              Rotation2d.fromDegrees(60.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(42.0), 
                                                            Units.inchesToMeters(285.0)),
                                          new Translation2d(Units.inchesToMeters(55.0), 
                                                            Units.inchesToMeters(304.0)),
                                          new Translation2d(Units.inchesToMeters(165.0), 
                                                            Units.inchesToMeters(305.0))),
                                  new Pose2d(Units.inchesToMeters(169.0),
                                             Units.inchesToMeters(306.0),
                                             Rotation2d.fromDegrees(0.0)),
                                  exitConfig
                                 );
    }

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    exitSpkrLeftToLeftWallCmd = new SwerveControllerCommand(exitSpkrLeftToLeftWall,
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
                new InstantCommand(()->swerveDrive.resetOdometry(exitSpkrLeftToLeftWall.getInitialPose())),
                exitSpkrLeftToLeftWallCmd,
                new InstantCommand(()->swerveDrive.stop())
               );
  }
}
