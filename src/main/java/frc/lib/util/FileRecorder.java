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
    private static boolean NOTE_LOGGING_ACTIVE;

    public FileRecorder(String filename, boolean isFileRecorderActive) {
        m_pathName = "/U/"+filename+".txt";
        if (isFileRecorderActive) {
            try {
                m_fileWriter = new FileWriter(m_pathName, false);   // Overwrite existing file
                m_bufferedWriter = new BufferedWriter(m_fileWriter);
            } catch (IOException e) {
                m_bufferedWriter = null;
                m_fileWriter = null;
                isFileRecorderActive = false;
            }
        } else {
            m_bufferedWriter = null;
            m_fileWriter = null;
        }
        NOTE_LOGGING_ACTIVE = isFileRecorderActive;
    }
     
    public static boolean isFileRecorderAvail() {
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
                m_bufferedWriter.write( caller+" Move: "+
                                        eventType.toString()+", "+
                                        F.df4.format(setpoint)+", "+
                                        F.df4.format(positionError)+", "+
                                        F.df80.format(relTimeMillis)+", "+
                                        F.df40.format(elapsedTime)+", "+
                                        state+", "+
                                        F.df20.format(seqNo) );
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
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
                m_bufferedWriter.write(caller+", "+
                                       requestType.toString()+", "+
                                       F.df4.format(setpoint)+", "+
                                       F.df80.format(startTimeMillis)+", "+
                                       state+", "+
                                       F.df20.format(seqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
            }
        }
    }

    public void recordIntakeEvent(NoteRequest requestType,
                                  long startTimeMillis,
                                  String state,
                                  int seqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write(requestType.toString()+", "+
                                       F.df80.format(startTimeMillis)+", "+
                                       state+", "+
                                       F.df20.format(seqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
            }
        }
    }

    public void recordShooterEvent( NoteRequest requestType,
                                    long startTimeMillis,
                                    double shooterRPS,
                                    double motorVoltage,
                                    double aimSetpoint,
                                    String currentState,
                                    int currentSeqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write(requestType.toString()+", "+
                                       F.df80.format(startTimeMillis)+", "+
                                       F.df4.format(shooterRPS)+", "+
                                       F.df4.format(motorVoltage)+", "+
                                       F.df4.format(aimSetpoint)+", "+
                                       currentState+", "+
                                       F.df20.format(currentSeqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
            }
        }
    };

    public void recordStateChange(long relTimeMillis,
                                  String oldState,
                                  String newState,
                                  int currentSeqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write( NoteEvent.STATE_CHANGE.toString()+", "+
                                        F.df80.format(relTimeMillis)+", "+
                                        oldState+", "+
                                        newState+", "+
                                        F.df2.format(currentSeqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
            }
        }
    }

    public void recordSeqNoChange(long relTimeMillis,
                                  String State,
                                  int oldSeqNo,
                                  int newSeqNo) {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.write( NoteEvent.SEQ_NO_CHANGE.toString()+","+
                                        F.df80.format(relTimeMillis)+", "+
                                        F.df20.format(oldSeqNo)+", "+
                                        F.df20.format(newSeqNo));
                m_bufferedWriter.newLine();
            } catch (IOException e) {
                closeFileRecorder();
            }
        }
    }

    public void closeFileRecorder() {
        if (NOTE_LOGGING_ACTIVE) {
            try {
                m_bufferedWriter.flush();
                m_bufferedWriter.close();
            } catch (Exception e) {
                // Ignore any exception - closing file anyway.
            }
        }
        m_bufferedWriter = null;
        m_fileWriter = null;
        NOTE_LOGGING_ACTIVE = false;
    }
}
