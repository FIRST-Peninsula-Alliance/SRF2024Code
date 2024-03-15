// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.SwerveSubsystem;

public class FinalScoot extends SequentialCommandGroup {
   private double autoM = AutoC.AUTO_WAYPOINT_MULTIPLIER;
  private SwerveSubsystem m_swerveDrive;
   private enum AutoStartAngle {
        AmpSide_60,
        SourceSide_60,
        StraightOn
      }
    public AutoStartAngle m_AutoStartAngle;

    public void publishAutoStartAngleData () {
    SmartDashboard.putString("AutoStartAngle ", m_AutoStartAngle.toString());
  }

  /** Creates a new FinalScoot. */
     public FinalScoot(SwerveSubsystem swerveDrive) {
    m_swerveDrive = swerveDrive;
    
    TrajectoryConfig configExit =
            new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                // .addConstraint(AutoConstants.autoVoltageConstraint);
        configExit.setReversed(false);

        // An example trajectory to follow, one path.  All units in meters. 

    Trajectory exitTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the -X direction, away from the
            // speaker goal. Position bumpers up against the subwoofer 
            // front, even if offset to one side (i.e. no angle shot with
            // this auto). Because the subwoofer extends ~1 m into the field,
            // the origin X is not really 0, but for now, consider it good.
            // On first test, a 2 m move resulted in closer to 8 m of movement. 
            // Need better tuning for the trajectory PIDs, but for now, set distance to
            // just .5 m, expecting about 2 m result, which should clear the zone.

            new Pose2d(0.886, 0.0, new Rotation2d(0.0)),
            List.of(
            new Translation2d(1.241*autoM, 0*autoM)),
            new Pose2d(1.241, 0.0, new Rotation2d(0.0)), //allowed range is pi/2 to -pi/2
            configExit);
            m_AutoStartAngle = AutoStartAngle.AmpSide_60;

      ProfiledPIDController thetaController =
          new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                    0,
                                    0,
                                    AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCmd =
          new SwerveControllerCommand(
              exitTrajectory,
              m_swerveDrive::getPose,
              SDC.SWERVE_KINEMATICS,
              new PIDController(AutoC.KP_X_CONTROLLER, 0, 0),
              new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
              thetaController,
              m_swerveDrive::setModuleStates,
              m_swerveDrive);

      addCommands(
            new InstantCommand(() -> m_swerveDrive.resetOdometry(exitTrajectory.getInitialPose())),
            swerveControllerCmd,
            new InstantCommand(()-> m_swerveDrive.stop())
            );
    }
  }

  
  
