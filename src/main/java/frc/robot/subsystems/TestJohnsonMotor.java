// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.JohnsonMotorConstants;
import frc.robot.Constants.LimitSwitchesConstants;

public class TestJohnsonMotor extends SubsystemBase {
  /** Creates a new TestJohnsonMotor. */
  public CANSparkMax m_JohnsonMotor = new CANSparkMax(JohnsonMotorConstants.kJohnsonMotorChannelID, MotorType.kBrushed);

  // This is for CANSparkMax Motor Controller if the encoder was in it
  // public RelativeEncoder m_JohnsonEncoder =
  // m_JohnsonMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 42);

  // This is for the encoder as a stand-alone and not connected to the CANSparkMax
  // class.
  public Encoder m_JohnsonEncoder = new Encoder(JohnsonMotorConstants.kJohnsonMotorPIDChannel1,
      JohnsonMotorConstants.kJohnsonMotorPIDChannel2, true, EncodingType.k1X);

  public PIDController m_PID = new PIDController(JohnsonMotorConstants.kP, JohnsonMotorConstants.kI,
      JohnsonMotorConstants.kD);

  public DigitalInput m_limit = new DigitalInput(LimitSwitchesConstants.kUpperLimitSwitchChannel);

  // Make sure to tell Matthew that this motor is Brushed and kQuadrature

  public TestJohnsonMotor() {
    SmartDashboard.putString("JohnsonMotor TicksPerRevolution", "44.4 hall pulses per rotation");

    // Configures the encoder's distance-per-pulse
    // The robot moves forward 1 foot per encoder rotation
    // There are 256 pulses per encoder rotation
    m_JohnsonEncoder.setDistancePerPulse(1. / 256.);

    m_JohnsonMotor.set(m_PID.calculate(m_JohnsonEncoder.getDistance(), Constants.JohnsonMotorConstants.kSetPoint));
    SmartDashboard.putNumber("PID Error", m_PID.getPositionError());
    SmartDashboard.putNumber("P from PID", m_PID.getP());
    SmartDashboard.putNumber("I from PID", m_PID.getI());
    SmartDashboard.putNumber("D from PID", m_PID.getD());
    SmartDashboard.putNumber("PID Calculation",
        m_PID.calculate(m_JohnsonEncoder.getDistance(), Constants.JohnsonMotorConstants.kSetPoint));

    // Sets the error tolerance to 5, and the error derivative tolerance to 10 per
    // second
    // m_PID.setTolerance(5, 10);

    // Returns true if the error is less than 5 units, and the
    // error derivative is less than 10 units
    m_PID.atSetpoint();

    // Resets PID + Encoder
    zeroPID();

    // Limits PID to Minimum and Maximum values
    JohnsonLimitations();
  }

  public void setMotor(double b) {
    m_JohnsonMotor.set(b);
  }

  public void stopJohnsonMotor() {
    m_JohnsonMotor.set(0);
  }

  public void zeroPID() {
    m_PID.reset();
    m_JohnsonEncoder.reset();
  }

  public void JohnsonLimitations() {
    m_PID.setIntegratorRange(Constants.JohnsonMotorConstants.kMIN_OUTPUT, Constants.JohnsonMotorConstants.kMAX_OUTPUT);
  }
  // set 0 at a cetain point
  // move it 45 degrees downawards
  // move it 45 degrees upwards

  // This would be the commands for the actual code
  public void MoveForward() {
    // If getDistance is less than 5 feet then the motor will keep running.
    // If it is greater than 5 feet then the motor will stop.
    if (m_JohnsonEncoder.getDistance() < 5) {
      m_JohnsonMotor.set(m_PID.calculate(m_JohnsonEncoder.getDistance(), Constants.JohnsonMotorConstants.kSetPoint));
      SmartDashboard.putString("Movement Stopped:", "False");
    } else {
      stopJohnsonMotor();
      zeroPID();
      SmartDashboard.putString("Movement Stopped:", "True");
    }
  }

  public void testMethod() {
    // If getDistance is less than negative 10 feet, then the motor will keep
    // running at 40% velocity.
    // If it is greater than 10 feet, then the motor will stop.
    if (m_JohnsonEncoder.getDistance() < -10) {
      m_JohnsonMotor.set(.4);
      SmartDashboard.putString("Movement Stopped:", "False");
    } else {
      stopJohnsonMotor();
      zeroPID();
      SmartDashboard.putString("Movement Stopped:", "True");
    }
  }

  public void autonomousPeriodic() {
    // If limit is false, run the motor backwards at half speed until the limit
    // switch is pressed
    // then turn off the motor and reset the encoder
    // This is for honing (otherwise known as homing)
    if (!m_limit.get()) {
      m_JohnsonMotor.set(0.5);
    } else {
      m_JohnsonMotor.set(0);
      zeroPID();
    }

  }
  // Equation will have to use circumference which is 2nr and then it will need
  // the diameter of the circle
  // Or we could guess and check the encoder values and see which one would be min
  // and max value limits,
  // so it won't go over the required amount.
  // Set encoder to a certain speed

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Test purposes:
    // MoveForward();
    // testMethod();

    // This is to get the encoder value
    SmartDashboard.putNumber("JohnsonMotor Encoder", m_JohnsonEncoder.getDistance());
    SmartDashboard.putBoolean("Upper Limit Switch", m_limit.get());
  }
}