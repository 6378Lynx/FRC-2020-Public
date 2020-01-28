package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class DriveSubsystem extends SubsystemBase {

    private final Talon leftTalon = new Talon(Constants.DriveConstants.leftMotor1Port);
    private final Victor leftVictor = new Victor(Constants.DriveConstants.leftMotor2Port);

    public DriveSubsystem(){

    }

    @Override
    public void periodic(){

    }


}
