package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {


    WPI_TalonSRX leftTalon, rightTalon;
    WPI_VictorSPX leftVictor, rightVictor;

    DifferentialDriveWheelSpeeds wheelSpeeds;
    DifferentialDriveKinematics kinematics;

    DifferentialDriveOdometry odometry;

    ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    double leftVelocity, rightVelocity;

    Pose2d pose;

    DifferentialDrive drive = new DifferentialDrive(leftTalon, rightTalon);

    public DriveSubsystem(){
        leftTalon = new WPI_TalonSRX(DriveConstants.leftMotor1Port);
        leftVictor = new WPI_VictorSPX(DriveConstants.leftMotor2Port);
        rightTalon = new WPI_TalonSRX(DriveConstants.rightMotor1Port);
        rightVictor = new WPI_VictorSPX(DriveConstants.rightMotor2Port);

        leftVictor.follow(leftTalon);
        rightVictor.follow(rightTalon);

        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        var speeds = new ChassisSpeeds();
        kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
        wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        leftVelocity = wheelSpeeds.leftMetersPerSecond;
        rightVelocity = wheelSpeeds.rightMetersPerSecond;

        odometry = new DifferentialDriveOdometry(getGyroHeading());
    }


    public void curvatureDrive(double fwd, double rot, boolean quickTurn){
        drive.curvatureDrive(fwd, rot, quickTurn);
    }

    public void arcadeDrive(double fwd, double rot){
        drive.arcadeDrive(fwd, rot);
    }

    public void setVoltage(double left, double right){
        leftTalon.setVoltage(left);
        rightTalon.setVoltage(right);
    }


    @Override
    public void periodic(){
        pose = odometry.update(getGyroHeading(), getLeftDistance(), getRightDistance());
    }


    public double getLeftDistance(){
        return leftTalon.getSelectedSensorPosition();
    }
    public double getRightDistance(){
        return rightTalon.getSelectedSensorPosition();
    }

    public double getLeftVelocity(){
        return leftTalon.getSelectedSensorVelocity();
    }

    public double getRightVelocity(){
        return rightTalon.getSelectedSensorVelocity();
    }

    public Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.gyroReversed ? -1.0 : 1.0));
    }

    public double getTurnRate(){
        return gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, getGyroHeading());
    }

    public void zeroHeading(){
        gyro.reset();
    }

}
