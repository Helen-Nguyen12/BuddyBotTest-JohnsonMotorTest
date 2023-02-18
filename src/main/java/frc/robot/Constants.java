/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int kLeftDriveMotorCANID = 1;
        public static final int kRightDriveMotorCANID = 2;
        public static final int kEncoderCountsPerRev = 1120;

        private static final double kWheelDiameter = 4; // wheel rolling diameter in inches
        public static final double kEncoderPositionConversionFactor = kWheelDiameter * Math.PI; // sets encoder to read
                                                                                                // position in inches

    }

    public static final class TestMotorsConstants {
        public static final int kTestMotor1PWMChannel = 0;
        public static final int kTestMotor2PWMChannel = 1;

    }

    public static final class PistonsConstants {
        public static final int kUpperPistonExtendChannel = 6;
        public static final int kUpperPistonRetractChannel = 7;
        public static final int kLowerPistonExtendChannel = 4;
        public static final int kLowerPistonRetractChannel = 5;

    }

    public static final class LimitSwitchesConstants {
        public static final int kUpperLimitSwitchChannel = 0;
        public static final int kLowerLimitSwitchChannel = 1;

    }

    public static final class JohnsonMotorConstants {
        public static final int kJohnsonMotorChannelID = 3;
        public static final int kJohnsonMotorPIDChannel1 = 4;
        public static final int kJohnsonMotorPIDChannel2 = 5;
        public static final double kP = 0.0002;
        public static final double kI = 0.000001;
        public static final double kD = 0.0004;
        public static final double kSetPoint = 30;
        public static final double kMIN_OUTPUT = -6000.0; // Maximum and minium range that PID can run in
        public static final double kMAX_OUTPUT = 6000.0;
        public static final int kLimitSwitchChannelID = 3;
    }
}
