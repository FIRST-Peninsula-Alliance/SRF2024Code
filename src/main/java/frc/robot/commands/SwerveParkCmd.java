// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

// This command sets the wheels to the "park: position",
// where it is really hard to move the robot. This could be 
// useful in two circumstances: 1. Avoid being pushed by 
// defending robot, or 2. Staying put once reasonably
// cetered on the charging station.
// The command remains active until one of the joystick
// suppliers exceeeds the dead-band tolerance.

public class SwerveParkCmd extends Command {
  private SwerveSubsystem m_swerveSubsystem;    
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private DoubleSupplier m_rotationSup;

  private double [] m_angles;

  public SwerveParkCmd (SwerveSubsystem swerveSubsystem, 
                        DoubleSupplier translationSup, 
                        DoubleSupplier strafeSup, 
                        DoubleSupplier rotationSup) {

    m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotationSup;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angles = new double[] { SDC.PARK_ANGLE_LEFT_DEG,
                              SDC.PARK_ANGLE_RIGHT_DEG,
                              -SDC.PARK_ANGLE_LEFT_DEG,
                              -SDC.PARK_ANGLE_RIGHT_DEG };
    m_swerveSubsystem.rotateModulesToAngles( m_angles );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when any joystick input becomes active. Note it is left to
    // the user via DefaultDriveCmd to set the next module directions;
    // the modules will stay "parked" with motors off until then.
    return ((Math.abs(m_translationSup.getAsDouble()) > UIC.JOYSTICK_DEADBAND)
            ||
            (Math.abs(m_strafeSup.getAsDouble()) > UIC.JOYSTICK_DEADBAND)
            ||
            (Math.abs(m_rotationSup.getAsDouble()) > UIC.JOYSTICK_DEADBAND));
  }
}
