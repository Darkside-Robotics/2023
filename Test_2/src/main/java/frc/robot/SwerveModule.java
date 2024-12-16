// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
private final RelativeEncoder m_driveEncoder;
  private final AnalogEncoder m_turningEncoder;

  // Offset for absolute encoder (must be calibrated occasionally) (during any large break)
  double turnEncoderOffest;
 
  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1.5, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          2,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   *
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turnEncoderPort) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    m_driveEncoder =  m_driveMotor.getEncoder();
    m_turningEncoder = new AnalogEncoder(turnEncoderPort);

    if(turnEncoderPort == 1){
      turnEncoderOffest = 0.8636; // Offset for right module (all black)
    }
    else{
      turnEncoderOffest = 0.151; // Offset for left module
    }

   
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity()*0.0007599, new Rotation2d(((m_turningEncoder.getAbsolutePosition()-turnEncoderOffest) *(2*Math.PI))));
  }
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(((m_turningEncoder.getAbsolutePosition()-turnEncoderOffest)*(2*Math.PI))));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) { // consider finding a way to add offset before it gets here
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(((m_turningEncoder.getAbsolutePosition()-turnEncoderOffest)*(2*Math.PI))));


    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity()*0.0007599, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    // Setpoint is modified with the encoder offset for the respective module (there is probably an easier way to do this)
    final double turnOutput =
        m_turningPIDController.calculate((((m_turningEncoder.getAbsolutePosition()-turnEncoderOffest)*(2*Math.PI))), (state.angle.getRadians()));

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
