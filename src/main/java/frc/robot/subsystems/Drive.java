/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
  /**
   * Creates a new Drive.
   */
  private final CANSparkMax m_LeftMotor = new CANSparkMax(DriveConstants.kLeftDriveMotorCANID,MotorType.kBrushed);
  private final CANSparkMax m_RightMotor = new CANSparkMax(DriveConstants.kRightDriveMotorCANID,MotorType.kBrushed);
  private final RelativeEncoder m_LeftEncoder = m_LeftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, DriveConstants.kEncoderCountsPerRev);
  private final RelativeEncoder m_RightEncoder = m_RightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, DriveConstants.kEncoderCountsPerRev);
  private final DifferentialDrive m_DriveController;
 


  public Drive() {
    m_LeftMotor.setInverted(true);
    m_RightMotor.setInverted(true);
    m_DriveController = new DifferentialDrive(m_LeftMotor, m_RightMotor);
    m_LeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderPositionConversionFactor);
    m_RightEncoder.setPositionConversionFactor(DriveConstants.kEncoderPositionConversionFactor);
    zeroEncoders();

  
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("Right Encoder", m_RightEncoder.getPosition());
      SmartDashboard.putNumber("Left Encoder", m_LeftEncoder.getPosition());
      SmartDashboard.putNumber("Average Encoder", getAverageEncoderDistance());


  }
  public void driveArcade(double xSpeed, double zRotation) {
    m_DriveController.arcadeDrive(xSpeed, zRotation);


  }

  public void driveTank(double leftSpeed, double rightSpeed) {
    m_DriveController.tankDrive(leftSpeed, rightSpeed);


  }

  public void stop() {
    m_LeftMotor.set(0);
    m_RightMotor.set(0);
  }

  public void zeroEncoders() {
    m_LeftEncoder.setPosition(0);
    m_RightEncoder.setPosition(0);

  }

  public double getAverageEncoderDistance() {
    return (m_LeftEncoder.getPosition() + m_RightEncoder.getPosition()) / 2;
  }

}
