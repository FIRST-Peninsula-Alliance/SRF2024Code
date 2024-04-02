package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

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
    private ClimbSubsystem           m_climbSubsystem;

    private SwerveParkCmd            m_parkCmd;

    // Declare handles for choosable autonomous Commands
    private JustScoreLeftAuto           m_justScoreLeftAuto;
    private JustScoreCenterAuto         m_justScoreCenterAuto;
    private JustScoreRightAuto          m_justScoreRightAuto;
    private ScoreThenExitRedLeftAuto    m_scoreThenExitRedLeftAuto;
    private ScoreThenExitRedRightAuto   m_scoreThenExitRedRightAuto;
    private ScoreThenExitBlueLeftAuto   m_scoreThenExitBlueLeftAuto;
    private ScoreThenExitBlueRightAuto  m_scoreThenExitBlueRightAuto;
    private Score2NotesLeftAuto         m_score2NotesLeftAuto;
    private Score2NotesCenterAuto       m_score2NotesCenterAuto;
    private Score2NotesRightAuto        m_score2NotesRightAuto;
    private ScoreMultipleNotesCenterAuto m_scoreMultipleNotesCenAuto;

    // Create sendable choosers for starting position and desired Auto routine
    private static SendableChooser<Command> m_autoRoutineChooser = new SendableChooser<>();

    // Declare CommandXboxController
    private static CommandXboxController m_xbox;

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        m_xbox = new CommandXboxController(0);

        m_swerveSubsystem = new SwerveSubsystem();
        m_masterArmSubsystem = new MasterArmSubsystem();
        m_climbSubsystem = new ClimbSubsystem();

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
                    
        m_scoreThenExitRedLeftAuto      = new ScoreThenExitRedLeftAuto(m_masterArmSubsystem,
                                                                       m_swerveSubsystem);
        m_scoreThenExitBlueLeftAuto     = new ScoreThenExitBlueLeftAuto(m_masterArmSubsystem,
                                                                        m_swerveSubsystem);
        m_scoreThenExitRedRightAuto     = new ScoreThenExitRedRightAuto(m_masterArmSubsystem,
                                                                        m_swerveSubsystem);
        m_scoreThenExitBlueRightAuto    = new ScoreThenExitBlueRightAuto(m_masterArmSubsystem,
                                                                         m_swerveSubsystem);
        m_score2NotesLeftAuto           = new Score2NotesLeftAuto(m_masterArmSubsystem, 
                                                                  m_swerveSubsystem);
        m_score2NotesCenterAuto         = new Score2NotesCenterAuto(m_masterArmSubsystem, 
                                                                    m_swerveSubsystem);
        m_score2NotesRightAuto          = new Score2NotesRightAuto(m_masterArmSubsystem, 
                                                                   m_swerveSubsystem);
        m_justScoreLeftAuto             = new JustScoreLeftAuto(m_masterArmSubsystem);
        m_justScoreCenterAuto           = new JustScoreCenterAuto(m_masterArmSubsystem);
        m_justScoreRightAuto            = new JustScoreRightAuto(m_masterArmSubsystem);
        m_scoreMultipleNotesCenAuto     = new ScoreMultipleNotesCenterAuto(m_masterArmSubsystem,
                                                                           m_swerveSubsystem);

        m_autoRoutineChooser.setDefaultOption("Score 2 Notes Center", m_score2NotesCenterAuto);
        m_autoRoutineChooser.addOption("Score 2 Notes Left", m_score2NotesLeftAuto);
        m_autoRoutineChooser.addOption("Score 2 Notes Right", m_score2NotesRightAuto);
        m_autoRoutineChooser.addOption("Score Center, then idle", m_justScoreCenterAuto);
        m_autoRoutineChooser.addOption("Score Left, then idle", m_justScoreLeftAuto);
        m_autoRoutineChooser.addOption("Score Right, then idle", m_justScoreRightAuto);
        m_autoRoutineChooser.addOption("Score Left, RED exit", m_scoreThenExitRedLeftAuto);
        m_autoRoutineChooser.addOption("Score Left, BLUE exit", m_scoreThenExitBlueLeftAuto);
        m_autoRoutineChooser.addOption("Score Right, RED exit", m_scoreThenExitRedRightAuto);
        m_autoRoutineChooser.addOption("Score Right, BLUE exit", m_scoreThenExitBlueRightAuto);
        m_autoRoutineChooser.addOption("Score Multiple Center", m_scoreMultipleNotesCenAuto);
        SmartDashboard.putData("Autonomous Selection:", m_autoRoutineChooser);

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
        ALT.and(m_xbox.back()).onTrue(new InstantCommand(()->m_climbSubsystem.overrideEndOfMatchSafety()));
        ALT.and(m_xbox.povUp()).onTrue(new InstantCommand(()-> m_climbSubsystem.raiseElevator()));
        ALT.and(m_xbox.povUp()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopElevator()));
        ALT.and(m_xbox.povDown()).onTrue(new InstantCommand(()-> m_climbSubsystem.lowerElevator()));
        ALT.and(m_xbox.povDown()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopElevator()));
        ALT.and(m_xbox.leftTrigger()).onTrue(new InstantCommand(()-> m_climbSubsystem.runClimbWinch()));
        ALT.and(m_xbox.leftTrigger()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopClimbWinch()));

    }

    /*
     * getSelectedAutoCommand is called from Robot.AutonomousInit(),
     * so if a good place to also check which position was selected for
     * the start of match, and if left or right is selected, then we can 
     * initialize the gyro to either 60.0 (for left), or 300.0 degrees
     * (for right).
     * The default is for the gyro to boot to 0 degrees, so no need to cover
     * any case other than left or right start position for setting the gyro.
     */
    public Command getSelectedAutoCommand() {
        Command selectedAuto = m_autoRoutineChooser.getSelected();

        if (selectedAuto == null) {
            selectedAuto = new DoNothingCmd();
        }
    
        if ((selectedAuto == m_justScoreLeftAuto)
            ||
            (selectedAuto == m_scoreThenExitRedLeftAuto)
            ||
            (selectedAuto == m_scoreThenExitBlueLeftAuto)
            ||
            (selectedAuto == m_score2NotesLeftAuto)) {
            m_swerveSubsystem.setGyro(60.0);
        } else if ((selectedAuto == m_justScoreRightAuto)
                   ||
                   (selectedAuto == m_scoreThenExitRedRightAuto)
                   ||
                   (selectedAuto == m_scoreThenExitBlueRightAuto)
                   ||
                   (selectedAuto == m_score2NotesRightAuto)) {
            m_swerveSubsystem.setGyro(300.0);
        } 
        return selectedAuto;
    }

    public void teleopStart() {
        m_masterArmSubsystem.teleopStart();
    }
}