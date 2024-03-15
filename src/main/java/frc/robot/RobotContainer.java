package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.Constants.*;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.ClimbSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystem local object handles */
    private SwerveSubsystem          m_swerveSubsystem;
    private MasterArmSubsystem       m_masterArmSubsystem;
    // private ClimbSubsystem           m_climbSubsystem;

    private final SwerveParkCmd     m_parkCmd;

    // Create SmartDashboard chooser for autonomous routines
    private DoNothingCmd m_doNothingAuto;
    private ScoreAndExitAuto m_scoreAndExitAuto;
    private JustExitCmd m_justExitAuto;
    // private Score2NotesAuto m_score2NotesAuto;
    private ScoreIndexedSpeakerCmd m_justScoreAuto;
    // private TestSquareAuto m_testSquareAuto;
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Xbox Controllers
    private static CommandXboxController m_xbox;

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        m_xbox = new CommandXboxController(0);

        m_swerveSubsystem = new SwerveSubsystem();
        m_masterArmSubsystem = new MasterArmSubsystem();
        // m_climbSubsystem = new ClimbSubsystem();

        m_swerveSubsystem.setDefaultCommand(
                new DefaultDriveCmd(m_swerveSubsystem,
                // Xbox stick forward and stick to left are both neg, so must negate all for WPILib expected coordinates:
                                    () -> -m_xbox.getLeftY(),    // translate: +fore / -back
                                    () -> -m_xbox.getLeftX(),    // strafe: +left / -right
                                    () -> -m_xbox.getRightX())); // rotate: +CCW / -CW

        m_parkCmd = new SwerveParkCmd(m_swerveSubsystem,
                                      () -> -m_xbox.getLeftY(),
                                      () -> -m_xbox.getLeftX(),
                                      () -> -m_xbox.getRightX());
                    
        m_doNothingAuto = new DoNothingCmd();
        m_scoreAndExitAuto = new ScoreAndExitAuto(m_masterArmSubsystem,
                                                  m_swerveSubsystem);
        m_justExitAuto = new JustExitCmd(m_swerveSubsystem);
        m_justScoreAuto = new ScoreIndexedSpeakerCmd(m_masterArmSubsystem);
        // m_score2NotesAuto = new Score2NotesAuto(m_masterArmSubsystem, 
        //                                         m_swerveSubsystem);
        // m_testSquareAuto = new TestSquareAuto(m_swerveSubsystem);
     
        m_chooser.setDefaultOption("Score, then Exit", m_scoreAndExitAuto);
        m_chooser.addOption("Do Nothing", m_doNothingAuto);
        m_chooser.addOption("Just Score", m_justScoreAuto);
        m_chooser.addOption("Just Exit", m_justExitAuto);
        //m_chooser.addOption("Score 2 Notes", m_score2NotesAuto);
        //m_chooser.addOption("Auto Square patterns", m_testSquareAuto);
        SmartDashboard.putData("Autonomous Selection: ", m_chooser);

        configureButtonBindings();
    }
    
    /**************************************************************
     * Getters for all subsystem Classes and other useful objects
     **************************************************************/
    public static XboxController getHidXboxCtrl(){
        return m_xbox.getHID();
    }

    /***********************************************
     * Button Bindings defines the operator UI
     ***********************************************/
    private void configureButtonBindings() {
        final Trigger ALT = m_xbox.leftBumper();

        // The following govern the UI for the Cresendo Season:
        
        
        //    Left Bumbper          => ALT mode for oter buttons when held. No action on its own
        //    Right Bumper          => Slow mode when held
        //    ALT + Right bumper    => super slow when held
        //    b                     => Pickup Note (full auto)
        //    y                     => Prep for indexed scoring in Speaker
        //    ALT + y               => Prep for distance scoring in Speaker
        //    a                     => Prep for scoring in Amplifier
        //    RightTrigger          => Shoot (score) when all checks are satisfied. 
        //                             The last a or y button determines which goal. 
        //    ALT + RightTrigger    => Discard note as soon as possible, ensure ejection outside robot
        //    x                     => Park (crossed wheel angles)
        //    ALT + x               => Cancel any Note handling action in progress. Retract arm(s) if out.
        //    L Joystick Y          => Swerve Translate (move fore/aft)
        //    L Joystick X          => Swerve Strafe (move side to side)
        //    R Joystick X          => Swerve Rotate (left = CCW, right = CW)
        //    R Joystick Y          => Maybe use for camera assisted side to side swerve positioning?
        //                             (Use Joystick axis just to set direction to look for April Tag? Or to set desired note, 
        //                              if more than one is visible on the floor?)
        //    L Joystick Button     => Set Field Oriented drive
        //    R Joystick Button     => Set Robot Oriented drive
        //    Back                  => Zero the Gyro
        //    Start                 => Single step through Note states if in debug mode
        //    ALT + Start           => Close Note Handler file logging.
        //    ALT + Back            => Simulate "end of match" period
        //    Left Trigger          => Simulate note acquired
        //    ALT + POV_UP          => extend elevator @ fixed speed when held (can get slower speed with Right Bumper)
        //    ALT + POV_DOWN        => retract elevator when held
        //    ALT + LeftTrigger     => Run climbing winch when held

        /*
        The following methods can be used to trigger rotation about any given corner. Most useful when
        playing defense. Retained here in case a quick change to defense at competition
        is needed, but normally these button bindings are needed for offense.
        m_xbox.leftTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFLCenOfRotation()));
        m_xbox.leftTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFRCenOfRotation()));                                        
        m_xbox.rightTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));  
        m_xbox.leftBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBLCenOfRotation()));
        m_xbox.leftBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBRCenOfRotation()));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        */
        // Left and right joystick buttons determine field oriented or robot oriented driving
        m_xbox.leftStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        m_xbox.rightStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        m_xbox.back().and(ALT.negate()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));   // was resetModulesToAbsolute()));

        // Right bumper alone = slow mode.
        // Alt + Right Bumper = very slow mode.
        // On Right bumper release (regardless of Left Bumper state), full speed.
        m_xbox.rightBumper().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.5)));
        ALT.and(m_xbox.rightBumper()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.2)));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(1.0)));

        m_xbox.x().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.discardNote()));  // was cancelNoteAction(), but that was never used
        // Swerve park 
        ALT.and(m_xbox.x()).onTrue(m_parkCmd);
        
        // Note handling activities
        m_xbox.b().onTrue(new InstantCommand(()->m_masterArmSubsystem.acquireNote()));
        m_xbox.a().onTrue(new InstantCommand(()->m_masterArmSubsystem.prepForAmpScore()));
        m_xbox.y().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.prepForIndexedSpeakerScore()));
        ALT.and(m_xbox.y()).onTrue(new InstantCommand(()->m_masterArmSubsystem.prepForDistantSpeakerScore()));
        m_xbox.rightTrigger(0.5).and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.scoreNote()));
        ALT.and(m_xbox.rightTrigger(0.5)).onTrue(new InstantCommand(()->m_masterArmSubsystem.discardNote()));
        m_xbox.start().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.stepPastDebugHold()));
        ALT.and(m_xbox.start()).onTrue(new InstantCommand(()->m_masterArmSubsystem.closeRecording()));
        m_xbox.leftTrigger().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.simulateNoteAcquired()));
    
        // Utilities for fine tuning various arm positions, can be used in extremis
        // during a match if pickup angles are not working.
        // Use povRight, povLeft, povUp and povDown for fine setpoint adjustments, 
        // about 1 degree per button press)
        m_xbox.povLeft().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustInnerArmSetpointUp()));
        m_xbox.povRight().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustInnerArmSetpointDown()));
        m_xbox.povUp().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustMasterArmSetpointUp()));
        m_xbox.povDown().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustMasterArmSetpointDown()));

        // climb activities all require ALT button combination
        /*
        ALT.and(m_xbox.back()).onTrue(new InstantCommand(()->m_climbSubsystem.overrideEndOfMatchSafety()));
        ALT.and(m_xbox.povUp()).onTrue(new InstantCommand(()-> m_climbSubsystem.raiseElevator()));
        ALT.and(m_xbox.povUp()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopElevator()));
        ALT.and(m_xbox.povDown()).onTrue(new InstantCommand(()-> m_climbSubsystem.lowerElevator()));
        ALT.and(m_xbox.povDown()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopElevator()));
        ALT.and(m_xbox.leftTrigger()).onTrue(new InstantCommand(()-> m_climbSubsystem.runClimbWinch()));
        ALT.and(m_xbox.leftTrigger()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopClimbWinch()));
        */
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}