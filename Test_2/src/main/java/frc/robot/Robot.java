// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;



import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final XboxController m_xbox = new XboxController(0);
    private final Drivetrain m_swerve = new Drivetrain();
  
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit(){

    }
  
    @Override
    public void teleopPeriodic() {
    

      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      final var xSpeed =
          -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_xbox.getLeftY(), 0.02))
              * Drivetrain.kMaxSpeed;
  
      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      // latest change: removed - sign because wgels werepoint the wrong way.
      final var ySpeed =
          m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_xbox.getLeftX(), 0.02))
              * Drivetrain.kMaxSpeed;
  
      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positive in
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      final var rot =
          -m_rotLimiter.calculate(MathUtil.applyDeadband(m_xbox.getRightX(), 0.02))
              * Drivetrain.kMaxAngularSpeed;
  
      m_swerve.drive(xSpeed, ySpeed, rot);
    }
  }
