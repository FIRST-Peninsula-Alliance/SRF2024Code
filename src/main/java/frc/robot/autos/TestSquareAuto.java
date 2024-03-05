
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants.*;
// import frc.robot.commands.SwerveParkCmd;
import frc.robot.subsystems.*;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestSquareAuto extends SequentialCommandGroup {
    SwerveSubsystem m_swerve;
    private static final double CF = 1/1.29 / (2.54*100);

    public TestSquareAuto (SwerveSubsystem swerve) {
        m_swerve = swerve;

        TrajectoryConfig config =
            new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                // .addConstraint(AutoConstants.autoVoltageConstraint);
        config.setReversed(false);

        // An example trajectory to follow, composed into 3 sepaarate paths
        // to allow middle path to be reversed.  All units in meters.

    // Try traversing a square, with unchanged heading, then do it in reverse
    
    Trajectory exampleTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction, on the near right 
            // corner of the intended square (the first time through move 
            // counterclockwise)

            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1.0, 0.0).times(CF),  // Pass through 1M forward
                    new Translation2d(2.0, 0.0).times(CF),  // Waypt at 2M forward
                    new Translation2d(2.0, 1.0).times(CF),  // Pass thru 1M west
                    new Translation2d(2.0, 2.0).times(CF),  // Waypt at 2M, 2M
                    new Translation2d(1.0, 2.0).times(CF),  // back up thru 1M
                    new Translation2d(0.0, 2.0).times(CF),  // Waypt at 0, 2M
                    new Translation2d(0.0, 1.0).times(CF)), // Move east thru 1M
            // And end where we started, facing forward
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            config);

    Trajectory exampleTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction, on the right side of
            // the intended square. This will be executed after exampleTrajectory1
            // (which is a CCW movement) so setup exampleTrajectory2 to move CW
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            List.of(new Translation2d(0.0, 1.0),  // first moving W
                    new Translation2d(0.0, 2.0),
                    new Translation2d(1.0, 2.0),  // then N
                    new Translation2d(2.0, 2.0),
                    new Translation2d(2.0, 1.0),  // then E
                    new Translation2d(2.0, 0.0),
                    new Translation2d(1.0, 0.0)), // and finally back S
            // End where we started, facing forward
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                       config);

        ProfiledPIDController thetaController =
            new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                      0,
                                      0,
                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                exampleTrajectory1,
                m_swerve::getPose,
                SDC.SWERVE_KINEMATICS,
                new PIDController(AutoC.KP_X_CONTROLLER, 0, 0),
                new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                m_swerve::setModuleStates,
                m_swerve);

        SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                exampleTrajectory2,
                m_swerve::getPose,
                SDC.SWERVE_KINEMATICS,
                new PIDController(AutoC.KP_X_CONTROLLER, 0, 0),
                new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                m_swerve::setModuleStates,
                m_swerve);

        addCommands(
            new InstantCommand(() -> m_swerve.resetOdometry(exampleTrajectory1.getInitialPose())),
            swerveControllerCommand1,
            new WaitCommand(1.0),
            swerveControllerCommand2,
            new WaitCommand(1.0),
            new InstantCommand(()-> m_swerve.stop())
            // m_park 
            );
    }
}