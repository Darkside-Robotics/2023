// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 1.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_LeftLocation = new Translation2d(0, 0.1778);
  private final Translation2d m_RightLocation = new Translation2d(0, -0.1778);

  private final SwerveModule m_Left = new SwerveModule(6, 5, 0);
  private final SwerveModule m_Right = new SwerveModule(7, 8, 1);
 
  static ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_LeftLocation, m_RightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
         Rotation2d.fromDegrees(-m_gyro.getAngle()),
          new SwerveModulePosition[] {
            m_Left.getPosition(),
            m_Right.getPosition(),
          });

  public Drivetrain() {
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, double periodSeconds) {

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(-m_gyro.getAngle()))
        );
                
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_Left.setDesiredState(swerveModuleStates[0]);
    m_Right.setDesiredState(swerveModuleStates[1]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(360-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_Left.getPosition(),
          m_Right.getPosition(),
        });
  }
}