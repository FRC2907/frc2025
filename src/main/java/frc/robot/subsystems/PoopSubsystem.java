package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class PoopSubsystem extends SubsystemBase{
    private SparkMax shoot;
    private SparkMaxConfig config;
    public boolean coralShot;

    public PoopSubsystem(){
        shoot = new SparkMax(Ports.manipulator.CORAL_SHOOTER, Control.coralManipulator.MOTOR_TYPE);
        config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
              .idleMode(IdleMode.kBrake)
              .inverted(false);
        shoot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void shoot(){
        shoot.set(Control.coralManipulator.kShootSpeed);
    }
    public void stop(){
        shoot.set(Control.coralManipulator.kStopSpeed);
    }


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
