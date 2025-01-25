package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class PoopSubsystem extends SubsystemBase{
    private SparkMax shoot;

    public PoopSubsystem(){
        shoot = new SparkMax(Ports.manipulator.CORAL_SHOOTER, Control.manipulator.MOTOR_TYPE);
    }

    public void shoot(){
        shoot.set(Control.manipulator.kShootSpeed);
    }
    public void stop(){
        shoot.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
