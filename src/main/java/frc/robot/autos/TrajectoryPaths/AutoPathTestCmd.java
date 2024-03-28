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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.WaitForMillisecsCmd;
import frc.robot.subsystems.SwerveSubsystem;

// The Auto Cmd is intended to create a simulated path for the robot to
// follow to pick up a note from the field, then return to the speaker goal
// and score that note. The path should be short enough to test in the lab,
// with twists and turns in both segments to make the test challenging.
public class AutoPathTestCmd extends SequentialCommandGroup {
    SwerveSubsystem m_swerve;
    CommandXboxController m_xbox;
 
    public AutoPathTestCmd(SwerveSubsystem swerve) {
        m_swerve = swerve;
        RumbleCmd rumbleLeftCmd = new RumbleCmd(1, 0.4, 500);
        RumbleCmd rumbleBothCmd = new RumbleCmd(3, 0.2, 1000);

        TrajectoryConfig config =
            new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                // .addConstraint(AutoConstants.autoVoltageConstraint);
        config.setReversed(false);
        TrajectoryConfig configReversed = config;
        configReversed.setReversed(true);

        ProfiledPIDController thetaController =
                                new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                          AutoC.KI_THETA_CONTROLLER,
                                                          0,
                                                          AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // The example trajectory to follow is composed of 3 separate paths
        // to allow actions between, and for the middle path to be reversed.
        // All units in meters.

        Trajectory curveTest1 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction, 
                // the curve will be more toward the left of the robot.
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),   // origin, facing forward
                List.of(new Translation2d(0.624, 0.0),              // first point, moving forward
                        new Translation2d(0.856, 0.033),                // first curve
                        new Translation2d(1.074, 0.163),
                        new Translation2d(1.202, 0.324),
                        new Translation2d(1.277, 0.571), 
                        new Translation2d(1.260, 0.783),
                        new Translation2d(1.175, 0.974),
                        new Translation2d(1.044, 1.117),
                        new Translation2d(0.894, 1.207),
                        new Translation2d(0.736, 1.250)),
                // End of first path, facing forward
                new Pose2d(0.5, 1.25, Rotation2d.fromDegrees(0.0)),
                config);

        Trajectory curveTest2 =
            TrajectoryGenerator.generateTrajectory(
                // Start Pose must be the same as the current location of the 
                // Robot, i.e. the same  as the end of curveTest1 
                new Pose2d(0.5, 1.25, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(0.75, 1.0),
                        new Translation2d(1.0, 0.75),
                        new Translation2d(1.25, 0.5)),
                // End of 2nd curve
                new Pose2d(1.5, 0.25, Rotation2d.fromDegrees(0.0)),
                config);

        Trajectory curveTest3 =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.5, 0.25, Rotation2d.fromDegrees(0.0)),
                List.of(new Translation2d(1.25, 0.25),
                        new Translation2d(.75, 0.0),
                        new Translation2d(0.25, 0.0)),
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                configReversed);                                    
    
        SwerveControllerCommand swerveCurveTest1Cmd =
            new SwerveControllerCommand(
                curveTest1,
                m_swerve::getPose,
                SDC.SWERVE_KINEMATICS,
                new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
                new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                m_swerve::setModuleStates,
                m_swerve);

        SwerveControllerCommand swerveCurveTest2Cmd =
            new SwerveControllerCommand(
                curveTest2,
                m_swerve::getPose,
                SDC.SWERVE_KINEMATICS,
                new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
                new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                m_swerve::setModuleStates,
                m_swerve);

        SwerveControllerCommand swerveCurveTest3Cmd =
                new SwerveControllerCommand(
                    curveTest3,
                    m_swerve::getPose,
                    SDC.SWERVE_KINEMATICS,
                    new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
                    new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
                    thetaController,
                    m_swerve::setModuleStates,
                    m_swerve);

        addCommands(new InstantCommand(() -> m_swerve.resetOdometry(curveTest1.getInitialPose())),
                    swerveCurveTest1Cmd,
                    rumbleBothCmd,
                    swerveCurveTest2Cmd,            
                    new WaitForMillisecsCmd(m_swerve, 1000),
                    swerveCurveTest3Cmd,
                    rumbleLeftCmd,
                    new InstantCommand(()-> m_swerve.stop()));
    }
}