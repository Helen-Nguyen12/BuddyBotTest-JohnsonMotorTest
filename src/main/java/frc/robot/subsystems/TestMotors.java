/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants.TestMotorsConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class TestMotors extends SubsystemBase {
  /**
   * Creates a new TestMotors.
   */

   private final Spark testMotor1 = new Spark(TestMotorsConstants.kTestMotor1PWMChannel);
   private final PWMVictorSPX testMotor2 = new PWMVictorSPX(TestMotorsConstants.kTestMotor2PWMChannel);
 

  public TestMotors() {}
 
  public void setMotors(double m1Speed, double m2Speed) {
    testMotor1.set(m1Speed);
    testMotor2.set(m2Speed);

  }

  public void stop() {
    testMotor1.set(0);
    testMotor2.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
