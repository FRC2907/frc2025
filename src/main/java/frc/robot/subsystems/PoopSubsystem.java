package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class PoopSubsystem extends SubsystemBase{
    private SparkMax shoot;
    public boolean coralShot;

    public PoopSubsystem(){
        shoot = new SparkMax(Ports.manipulator.CORAL_SHOOTER, Control.coralManipulator.MOTOR_TYPE);
    }

    public void shoot(){
        shoot.set(Control.coralManipulator.kShootSpeed);
    }
    public void stop(){
        shoot.set(Control.coralManipulator.kStopSpeed);
    }

    /*private void currentDetection(){
        if (shoot.getOutputCurrent() > 15){
            coralShot = true;
        } else {
            coralShot = false;
        }
    }*/

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("current", shoot.getOutputCurrent());
        SmartDashboard.putBoolean("shot", coralShot);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
