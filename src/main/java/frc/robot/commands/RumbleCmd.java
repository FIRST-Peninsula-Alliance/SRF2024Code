package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.RobotContainer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RumbleCmd extends Command {
  private XboxController m_xbox;
  private int m_whichSide;
  private double m_amplitude;
  private long m_runDurationMillis;

  private long m_startTime;

  /** Creates a new RumbleCommand
   * Provides Game Controller tactile feedback to the operator.
   * Typically called from a subsystem when something is needing attention, 
   * such as an overheating motor or motor controller, an over current
   * situation (e.g. an individual motor stall, or a brownout), or when it 
   * is time to change the battery in non-competitive situations, like parades 
   * and demos. It could also be for some game specific feedback. 
   * Essentially it is up to the drivers and programmers to agree on the 
   * meaning in a given application.
   * Rumble left, right, or both, together with duration and aplitude, offer 
   * a potentially very large feedback "vocabulary". 
   * That said, it is good advice to keep it simple, and not assign more than 
   * two or three feedback signals per application.
   *
   * m_XboxHIDCtrl is a native HID Xbox controller object (has rumble motors), the
   * handle to which is getten by calling getHidXboxCtrl(), implemented in
   * RobotContainer.
   * 
   * @param int whichSide - 1 = left, 2 = right, 3 = both, anything else = neither
   * @param double amplitude (0 to 1) 0 = off, 1 = full power
   * @param long duration (in milliseconds)
   */
  public RumbleCmd(int whichSide, double amplitude, long durationTimeMs) {
    m_xbox = RobotContainer.getHidXboxCtrl();
    m_whichSide = whichSide;

    if ((amplitude >= 0.0) && (amplitude <= 1.0)) {
      m_amplitude = amplitude;
    } else {
      m_amplitude = 0.0;
    }

    if ((durationTimeMs > 0) && (durationTimeMs < 10000)) {
      m_runDurationMillis = durationTimeMs;
    } else {
      m_runDurationMillis = 0;
    }
  }

  // initialize is called once every time the command is scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    // tried using kBothRumble, did not seem to work, so drive each separately
    if ((m_whichSide == 1) || (m_whichSide == 3)) {
      m_xbox.setRumble(RumbleType.kLeftRumble, m_amplitude);
    }

    if ((m_whichSide == 2) || (m_whichSide == 3)) {
     m_xbox.setRumble(RumbleType.kRightRumble, m_amplitude);
    }
    //SmartDashboard.putString("Rumble "+m_whichSide, " Active");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_xbox.setRumble(RumbleType.kLeftRumble, 0.0);
    m_xbox.setRumble(RumbleType.kRightRumble, 0.0);
    //SmartDashboard.putString("Rumble "+m_whichSide, " Ended");
  }

  // Returns true when the command should end, in this case either via timeout, 
  // or DPAD Down.
  @Override
  public boolean isFinished() {
    return ((System.currentTimeMillis() - m_startTime) > m_runDurationMillis);
  }
}
