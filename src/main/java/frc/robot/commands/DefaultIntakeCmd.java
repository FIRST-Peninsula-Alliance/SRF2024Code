// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/*******************************************
 * This is only used for testing! It allows
 * Joystick or button input to run the intake
 * motor.
 * We don't even add any dependency but if we
 * did, it would need to be MasterArmSubsystem.
 *******************************************/
public class DefaultIntakeCmd extends Command {
  private IntakeSubsystem m_intakeSubsystem;
  private DoubleSupplier m_intakeSupplier;

  /** Creates a new DefaultIntakeCnd. */
  public DefaultIntakeCmd(IntakeSubsystem intake, DoubleSupplier intakeSupplier) {
    m_intakeSubsystem = intake;
    m_intakeSupplier = intakeSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotState.isEnabled() && RobotState.isTest()) {
      m_intakeSubsystem.testDrive(m_intakeSupplier);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
