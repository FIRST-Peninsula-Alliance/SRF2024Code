// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class ReturnToSpeakerCmd extends SequentialCommandGroup {
  private SwerveSubsystem m_swerveDrive;

  /** Creates a new ScoreAndMove. */
  public ReturnToSpeakerCmd(SwerveSubsystem swerveDrive) {
    m_swerveDrive = swerveDrive;

    TrajectoryConfig configMove =
            new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                // .addConstraint(AutoConstants.autoVoltageConstraint);
    configMove.setReversed(true);   // drive backwards

    Trajectory returnTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the end of the last trajectory, which should be the 
            // one defined in JustExitCmd
            new Pose2d(1.5, 0.0, Rotation2d.fromDegrees(0.0)),
            List.of(new Translation2d(1.25, 0.0),
                    new Translation2d(1.0, 0.0),
                    new Translation2d(0.75, 0.0),
                    new Translation2d(0.5, 0.0),
                    new Translation2d(0.25, 0.0)),
            new Pose2d(0.0, 0, Rotation2d.fromDegrees(0.0)),
            configMove);

      ProfiledPIDController thetaController =
          new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                    AutoC.KI_THETA_CONTROLLER,
                                    0,
                                    AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCmd =
          new SwerveControllerCommand(
              returnTrajectory,
              m_swerveDrive::getPose,
              SDC.SWERVE_KINEMATICS,
              new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
              new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
              thetaController,
              m_swerveDrive::setModuleStates,
              m_swerveDrive);

      addCommands(
                  // Just move, do not reset odometry!
                  swerveControllerCmd
                 );
    }
}
