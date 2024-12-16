// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 1.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_LeftWheel = new Translation2d(0, 0.1778);
  private final Translation2d m_RightWheel = new Translation2d(0, -0.1778);

  // Swerve Module is a function that takes (drive motor CAN ID, turn motor CAN ID, and turn encoder port)

  private final SwerveModule m_Left = new SwerveModule(6, 5, 0);
  private final SwerveModule m_Right = new SwerveModule(7, 8, 1);


  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_LeftWheel, m_RightWheel);



  public Drivetrain() {

  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot) {

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_Left.setDesiredState(swerveModuleStates[0]);
    m_Right.setDesiredState(swerveModuleStates[1]);
  }

}

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

