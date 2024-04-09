// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import frc.robot.Constants.F;

// This Class provides an optional logging function for the 2024 Cresendo Game 
// Note Handler Subsystems. State changes, action requests, events like reaching 
// setpoint,s or timeouts upon failing to reach setpoints, etc. can all be logged 
// in a file. The only file storage option is a USB thumb drive. If not present,
// or if some other IO error (such as disk full) occurs, FileRecording will be 
// cancelled. To avoid writing over old files, each specified filename is pre-
// pended with "/U/" (the default letter for single USB drives on RoboRio) and
// apended with a date and time string.

public class FileRecorder {

    public enum NoteEvent {
        SETPOINT_REACHED,
        TIMEOUT_OCCURED,
        SHOT_DETECTED,
        STATE_CHANGE,
        SEQ_NO_CHANGE
    }

    public enum NoteRequest {
        MOVE_MASTER_ARM,
        MOVE_INNER_ARM,
        ACQUIRE_PREP,
        INTAKE_ACQUIRE,
        INTAKE_HOLD,
        INTAKE_EJECT,
        INTAKE_STOP,
        SHOOTER_PREP,
        SHOOTER_READY,
        SHOOTER_SCORE,
        CANCEL_SHOT,
        AMP_PREP,
        AMP_READY,
        AMP_SCORE
    }

    private static FileWriter m_fileWriter;
    private BufferedWriter m_bufferedWriter;
    private String m_pathName;
    private String m_stringBuf;
    private String m_separator = ", ";
    private boolean NOTE_LOGGING_ACTIVE;

    public FileRecorder(String filename, boolean isFileRecorderActive) {
        m_pathName = "/U/"+filename+".txt";
        if (isFileRecorderActive) {
            try {
                m_fileWriter = new FileWriter(m_pathName, false);   // Overwrite existing file
                m_bufferedWriter = new BufferedWriter(m_fileWriter);
                System.out.println("File Recorder successfully started");
                m_stringBuf = "Record Type, Request, Curr SP, SP Err, Time, Elapsed Time, Curr State, New State, SeqNo, New SeqNo";
                m_bufferedWriter.write(m_stringBuf);
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                m_bufferedWriter = null;
                m_fileWriter = null;
                isFileRecorderActive = false;
                System.out.println("File Recorder Open of "+m_pathName+" Failed");
            }
        } else {
            m_bufferedWriter = null;
            m_fileWriter = null;
        }
        NOTE_LOGGING_ACTIVE = isFileRecorderActive;
    }
     
    public boolean isFileRecorderAvail() {
        return NOTE_LOGGING_ACTIVE;
    }

    public void recordMoveEvent(String caller,
                                NoteEvent eventType,
                                double setpoint,
                                double positionError,
                                long relTimeMillis,
                                long elapsedTime,
                                String state,
                                int seqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write( caller+m_separator+
                                        eventType.toString()+m_separator+
                                        F.df4.format(setpoint)+m_separator+
                                        F.df4.format(positionError)+m_separator+
                                        F.df80.format(relTimeMillis)+m_separator+
                                        F.df40.format(elapsedTime)+m_separator+
                                        state+m_separator+
                                        /* new state */ m_separator+
                                        F.df20.format(seqNo) );
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
                System.out.println("FR RecordMoveEvent Failed");
            }
        }
    }

    public void recordReqEvent(String caller,
                               NoteRequest requestType,
                               double setpoint,
                               long startTimeMillis,
                               String state,
                               int seqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write(caller+m_separator+
                                       requestType.toString()+m_separator+
                                       F.df4.format(setpoint)+m_separator+
                                       /* SP Err */ m_separator+
                                       F.df80.format(startTimeMillis)+m_separator+
                                       state+m_separator+
                                       /* new state */ m_separator+
                                       F.df20.format(seqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
                System.out.println("FR recordReqEvent failed");
            }
        }
    }

    public void recordIntakeEvent(NoteRequest requestType,
                                  double intakeMotorVelocity,
                                  long startTimeMillis,
                                  String state,
                                  int seqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write("Intake Req"+m_separator+
                                       requestType.toString()+m_separator+
                                       intakeMotorVelocity+m_separator+
                                       /* SP Err */  m_separator+
                                       F.df80.format(startTimeMillis)+m_separator+
                                       /* Elapsed time */ m_separator+
                                       state+m_separator+
                                       /* New state */ m_separator+
                                       F.df20.format(seqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
                System.out.println("FR recordIntakeEvent failed");
            }
        }
    }

    public void recordShooterEvent( NoteRequest requestType,
                                    double shooterRPS,
                                    double aimSetpoint,
                                    long startTimeMillis,
                                    String currentState,
                                    int currentSeqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write("Shooter"+m_separator+
                                       requestType.toString()+m_separator+
                                       F.df4.format(shooterRPS)+m_separator+
                                       F.df4.format(aimSetpoint)+m_separator+
                                       F.df80.format(startTimeMillis)+m_separator+
                                       currentState+m_separator+
                                       /* new state */ m_separator+
                                       F.df20.format(currentSeqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
                System.out.println("FR recordShooterEventfailed");
            }
        }
    };

    public void recordStateChange(long relTimeMillis,
                                  String oldState,
                                  String newState,
                                  int currentSeqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write( NoteEvent.STATE_CHANGE.toString()+m_separator+
                                        /* Event type moot */ m_separator+
                                        /* SP */ m_separator+
                                        /* SP Err */ m_separator+
                                        F.df80.format(relTimeMillis)+
                                        /* elapsed time */ m_separator+
                                        oldState+m_separator+
                                        newState+m_separator+
                                        F.df2.format(currentSeqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
                System.out.println("FR recordStateChange failed");
            }
        }
    }

    public void recordSeqNoChange(long relTimeMillis,
                                  String State,
                                  int oldSeqNo,
                                  int newSeqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write( NoteEvent.SEQ_NO_CHANGE.toString()+m_separator+
                                        /* Event type moot */ m_separator+
                                        /* SP */ m_separator+
                                        /* SPErr */ m_separator+
                                        F.df80.format(relTimeMillis)+m_separator+
                                        /* Elapsed time */ m_separator+
                                        State+m_separator+
                                        /* new state */ m_separator+
                                        F.df20.format(oldSeqNo)+m_separator+
                                        F.df20.format(newSeqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
                System.out.println("FR recordSeqNoCHange failed");
            }
        }
    }

    public void closeFileRecorder() {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.flush();
                m_bufferedWriter.close();
                System.out.println("File Recorder successfully closed");
            } catch (Exception e) {
                System.out.println("FR Close failed");
                // Ignore any exception - closing file anyway.
            }
        }
        m_bufferedWriter = null;
        m_fileWriter = null;
        NOTE_LOGGING_ACTIVE = false;
    }
}
