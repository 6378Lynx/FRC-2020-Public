/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants{
        public static final int leftMotor1Port = 0;
        public static final int leftMotor2Port = 0;

        public static final int rightMotor1Port = 0;
        public static final int rightMotor2Port = 0;

        public static final int leftEncoderPort = 0;
        public static final int rightEncoderPort = 0;

        public static final int gyroPort = 0;

        public static final int trackWidth = 0;

        public static final boolean gyroReversed = false;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kPDrive = 0.0;

        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);

    }

    public static final class OIConstants{
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 0;

        static final int DRIVER_A_Button = 2;
        static final int DRIVER_B_Button = 3;
        static final int DRIVER_X_Button = 1;
        static final int DRIVER_Y_Button = 4;
        static final int DRIVER_leftBumper = 5;
        public static final int DRIVER_rightBumper = 6;
        public static final int DRIVER_backButton = 7;
        public static final int DRIVER_startButton = 8;

        //XBOX Controller / OPERATOR PORTS

        static final int OPERATOR_A_Button = 1;
        static final int OPERATOR_B_Button = 2;
        static final int OPERATOR_X_Button = 3;
        static final int OPERATOR_Y_Button = 4;
        static final int OPERATOR_leftBumper = 5;
        static final int OPERATOR_rightBumper = 6;
        static final int OPERATOR_backButton = 7;
        static final int OPERATOR_startButton = 8;
    }

    public static final class AutoConstants{
        public static final double maxVel = 0;
        public static final double maxAccel = 0;
    }

    public static final class ShooterConstants{
        public static final int lowerLimitSwitchPort = 0;
        public static final int upperLimitSwitchPort = 1;

        public static final int shooterMotorPort = 0;
        public static final int rotationMotorPort = 1;

    }

    public static final class ControlConstants{
        public static final int raiseMotorPort = 0;
        public static final int turnMotorPort = 0;
        public static int topLimitSwitchPort = 0;
        public static int bottomLimitSwitchPort = 0;
    }

}
