// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;


public class SwerveMotors extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

 private final CANSparkMax m_driveMotor;
 private final CANSparkMax m_turnMotor;

 m_driveMotor = new CANSparkMax(OperatorConstants.DriveMotorID, MotorType.kBrushless);
 m_turnMototr = new CANSparkMax(OperatorConstants.TurnMototID, MotorType.kBrushless);

  public SwerveMotors() {}

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
