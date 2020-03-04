package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

    private Spark rotationMotor = new Spark(Constants.ShooterConstants.rotationMotorPort);
    private Spark shooterMotor =  new Spark(Constants.ShooterConstants.shooterMotorPort);
    private DigitalInput lowerLimitSwitch = new DigitalInput(Constants.ShooterConstants.lowerLimitSwitchPort);
    private DigitalInput upperLimitSwitch = new DigitalInput(Constants.ShooterConstants.upperLimitSwitchPort);


    @Override
    public void periodic(){

    }


    public void outtake(){
        shooterMotor.set(1);
    }

    public void intake(){
        shooterMotor.set(-1);
    }

    public void raiseShooter() {
        rotationMotor.set(0.8);
    }
    public void lowerShooter(){
        rotationMotor.set(-0.8);
    }

    public void stopRotation(){
        rotationMotor.set(0);
    }

    public void stopShooter(){
        shooterMotor.set(0);
    }

    public boolean getUpperLimitSwitch(){return upperLimitSwitch.get();}
    public boolean getLowerLimitSwitch(){return lowerLimitSwitch.get();}

}
