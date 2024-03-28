package frc.robot.subsystems;

import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private Pose2d m_location;
    private SwerveDriveOdometry m_swerveOdometry;
    private SwerveModule[] m_swerveMods;
    private SwerveModuleState[] m_states = new SwerveModuleState[4];
    private Pigeon2 m_gyro;
    private Translation2d m_cenOfRotationOffset = SDC.REL_POS2D_CEN;
    private boolean m_isFieldOriented = true;       // default is Field Oriented on start
    private static double m_varMaxOutputFactor = 1.0;      // A temporary driver settable speed 
                                                    // reduction factor, normally 1.0.
                                                    // when triggered (by Right Bumper)
                                                    // speed will be slower, for both 
                                                    // translate and strafing, and rotating
    private double m_fixedMaxTranslationOutput  = 
            SDC.OUTPUT_DRIVE_LIMIT_FACTOR;          // This and the following are fixed 
    private double m_fixedMaxRotationOutput     =   // (changable via re-compile only)
            SDC.OUTPUT_ROTATE_LIMIT_FACTOR;         // reductions in the max speeds
                                                    // allowed, to reduce chance of 
                                                    // damage, independent of
                                                    // m_varMaxOutputFactor.
    private GenericEntry        m_gyroYawEntry;
    private GenericEntry        m_gyroRawYawEntry;
    private GenericEntry        m_isFieldOrientedEntry;
    // private GenericEntry        m_gyroPitchEntry;
    // private GenericEntry        m_gyroRollEntry;
    private GenericEntry        m_odometryPoseXEntry;
    private GenericEntry        m_odometryPoseYEntry;
    private GenericEntry        m_odometryHeadingEntry;

    public SwerveSubsystem() {
        m_gyro = new Pigeon2(GC.PIGEON_2_CANID, Constants.ROBO_RIO_BUS_NAME);
        Pigeon2Configuration p2Config = new Pigeon2Configuration();
        m_gyro.getConfigurator().apply(p2Config);
        zeroGyro();

        SmartDashboard.putNumber("Chosen Module Circumference M =", SDC.WHEEL_CIRCUMFERENCE_M);

        m_swerveMods = new SwerveModule[] {
            new SwerveModule(0, SDC.FL_Mod0.MODULE_CONSTANTS),
            new SwerveModule(1, SDC.FR_Mod1.MODULE_CONSTANTS),
            new SwerveModule(2, SDC.BL_Mod2.MODULE_CONSTANTS),
            new SwerveModule(3, SDC.BR_Mod3.MODULE_CONSTANTS)
        };

        // By pausing init for a second before setting module offsets, we avoid 
        // a bug with inverting motors.
        // See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        Timer.delay(1.0);
        // Update 1/2024: also added waitForCANcoder method in SwerveModule, 
        // which does not hurt but according to above thread the delay is still 
        // needed, so restored that.
        resetModulesToAbsolute();

        m_swerveOdometry = new SwerveDriveOdometry(SDC.SWERVE_KINEMATICS, 
                                                   getYaw2d(), 
                                                   getModulePositions());
        setupPublishing();
    }

    // The following five methods establish the center of rotation, initially or
    // on the fly, to either the center of the robot (default) or to one of the 
    // four module wheels.
    public void setFLCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_FL;
    }

    public void setFRCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_FR;
    }

    public void setBLCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_BL;
    }

    public void setBRCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_BR;
    }

    public void resetCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_CEN;
    }

    // drive() is the handler for teleop joystick driving, typically called from 
    // DefaultDriveCmd with isOpenLoop set to false.
    // It can also be called from PID controllers or other Commands as needed, 
    // typically with isOpenLoop set to false.
    public void drive(Translation2d translation, 
                      double rotation, 
                      boolean isOpenLoop) {
        // Output pulse on DIO_1 for oscilloscope viewing
        translation = translation.times(m_varMaxOutputFactor * m_fixedMaxTranslationOutput);
        rotation = rotation * m_varMaxOutputFactor * m_fixedMaxRotationOutput;
       
        ChassisSpeeds chassisSpeeds = m_isFieldOriented 
                                        ? 
                                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                            translation.getX(), 
                                            translation.getY(), 
                                            rotation, 
                                            getYaw2d())
                                        :
                                        new ChassisSpeeds(
                                            translation.getX(), 
                                            translation.getY(), 
                                            rotation);
        SwerveModuleState[] swerveModuleStates =
            SDC.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds,
                                                       m_cenOfRotationOffset);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 
                                                    SDC.MAX_ROBOT_SPEED_M_PER_SEC);
        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.m_modNum], isOpenLoop);
        }
    }  

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SDC.MAX_ROBOT_SPEED_M_PER_SEC);
        
        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(desiredStates[mod.m_modNum], false);
        }
    }    

    public boolean isFieldOriented() {
        return m_isFieldOriented;
    }

    public void setFieldOriented( boolean fieldOrientedSetting ) {
        m_isFieldOriented = fieldOrientedSetting;
    }

    public void setVarMaxOutputFactor(double maxOutputFactor) {
        if (maxOutputFactor < .1) {
            maxOutputFactor = .1;
        }
        if (maxOutputFactor > 1.0) {
            maxOutputFactor = 1.0;
        };
        m_varMaxOutputFactor = maxOutputFactor;
    }

    public static double getVarMaxOutputFactor() {
        return m_varMaxOutputFactor;
    } 

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose2d) {
        m_swerveOdometry.resetPosition(getYaw2d(), getModulePositions(), pose2d);
    }

    public double getRobotTranslateVel() {
        m_states = getModuleStates();
        return (SDC.SWERVE_KINEMATICS
                .toChassisSpeeds(m_states).vxMetersPerSecond);
    }
    
    public double getRobotStrafeVel() {
        m_states = getModuleStates();
        return (SDC.SWERVE_KINEMATICS
                .toChassisSpeeds(m_states).vyMetersPerSecond);
    }
    
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveMods){
            states[mod.m_modNum] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveMods){
            positions[mod.m_modNum] = mod.getModulePosition();
        }
        return positions;
    }

    public void zeroGyro(){
        m_gyro.reset();
    }

    public void setGyro(double newHeadingDeg){
        m_gyro.setYaw(newHeadingDeg);
    }

    public Rotation2d  getYaw2d() {
        return (GC.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw().getValueAsDouble())
                                  : Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }
/*
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(m_gyro.getPitch().getValueAsDouble());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(m_gyro.getRoll().getValueAsDouble());        
    }
*/
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : m_swerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setupPublishing() {
        ShuffleboardTab sbt = Shuffleboard.getTab("SwerveDrive");
        if (sbt == null) {
            SmartDashboard.putString("SwerveDrive Tab", "getTab() Error occured");
        } else {
            ShuffleboardLayout sl = sbt.getLayout("RobotData", BuiltInLayouts.kList)
                                    .withPosition(0, 0)
                                    .withSize(2, SDC.SWERVE_MOD_LIST_HGT)
                                    .withProperties(Map.of("Label position", "LEFT"));
            if (sl == null) {
                SmartDashboard.putString("RobotData Layout", "getLayout() Error occured");
            } else {
                m_gyroYawEntry =        sl.add("Gyro Yaw", "0").getEntry();
                m_gyroRawYawEntry =     sl.add("Gyro Raw Yaw", "0").getEntry();
                m_isFieldOrientedEntry= sl.add("Field Oriented ", "Yes").getEntry();
                m_location = m_swerveOdometry.getPoseMeters();
                m_odometryPoseXEntry =  sl.add("Pose X", F.df2.format(m_location.getX())).getEntry();
                m_odometryPoseYEntry =  sl.add("Pose Y", F.df2.format(m_location.getY())).getEntry();
                m_odometryHeadingEntry = sl.add("Pose Heading", F.df2.format(m_location.getRotation().getRadians())).getEntry();
                // m_gyroPitchEntry = // opt
                // m_gyroRollEntry = // opt
                // m_cenOfRotationEntry = // opt
                // m_varMaxOutputEntry = // opt
                // m_odometrySpeedEntry = swerveSubsysLayout
                // m_maxMeasuredSpeedEntry = swerveSubsysLayout
                // m_maxMeasuredAccelEntry = swerveSubsysLayout
                if ((m_gyroYawEntry == null)
                    || (m_isFieldOrientedEntry == null)
                    || (m_odometryPoseXEntry == null)
                    || (m_odometryPoseYEntry == null)
                    || (m_odometryHeadingEntry == null)) {
                    SmartDashboard.putString("RobotData List Entries", "Null Entry handles(s) encountered");
                }
            }
        }
    }

    public void publishSwerveDriveData() {
            m_gyroYawEntry.setString(F.df1.format(getYaw2d().getDegrees()));
            m_gyroRawYawEntry.setString(F.df1.format(m_gyro.getYaw().getValueAsDouble()));
            m_odometryPoseXEntry.setString(F.df2.format(m_location.getX()));
            m_odometryPoseYEntry.setString(F.df2.format(m_location.getY()));           
            m_odometryHeadingEntry.setString(F.df2.format(m_location.getRotation().getRadians()));
            m_isFieldOrientedEntry.setString(m_isFieldOriented ? "Yes" : "No");
        // m_gyroPitchEntry
        // m_gyroRollEntry

        for(SwerveModule mod : m_swerveMods) {
            mod.publishModuleData();
        }
    }


    @Override
    public void periodic() {
        // Update odometry on every loop instance
        m_swerveOdometry.update(getYaw2d(), getModulePositions()); 
        m_location = m_swerveOdometry.getPoseMeters();
        publishSwerveDriveData();
    }

    // This is a test routine, designed to rotate all modules
    // synchronously to an identical specified heading in degrees
    public void rotateModulesToAngle(double angleDeg) {
        for(SwerveModule mod : m_swerveMods) {
            mod.setAngle(angleDeg);
        }    
    }

    // This is a method to cause all modules to be rotated to 
    // angles contained in an array passed as an argument, the
    // specified angles being in FL, FR, BL, and BR order.
    // Primarily used to set all modules to their PARK positions, 
    // but could be useful for other purposes.
    public void rotateModulesToAngles( double angleDeg[] ) {
        for(SwerveModule mod : m_swerveMods) {
            mod.setAngle(angleDeg[mod.m_modNum] );
        }
    }

    public void stop() {
        for(SwerveModule mod : m_swerveMods) {
            mod.stop();
        }            
    }
}