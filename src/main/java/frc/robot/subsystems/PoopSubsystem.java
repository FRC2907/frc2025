package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class PoopSubsystem extends SubsystemBase{
    private SparkMax shoot;
    private SparkMaxConfig config;
    private TimeOfFlight sensor;
    private boolean coralShot;

    private PoopSubsystem(){
        shoot = new SparkMax(Ports.manipulator.CORAL_SHOOTER, Control.coralManipulator.MOTOR_TYPE);
        config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
              .idleMode(IdleMode.kBrake)
              .inverted(false);
        shoot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sensor = new TimeOfFlight(Ports.manipulator.TOF_SENSOR);
    }

    private static PoopSubsystem instance;
    public static PoopSubsystem getInstance(){
        if (instance == null){
            instance = new PoopSubsystem();
        }
        return instance;
    }



    public void shoot(){
        shoot.set(Control.coralManipulator.kShootSpeed);
    }
    public void stop(){
        shoot.set(Control.coralManipulator.kStopSpeed);
    }

    public boolean hasCoral(){
        return sensor.getRange() < Control.coralManipulator.kProximityBand;
    }

    public Command coralWaitIntakeCommand(){
        return new WaitUntilCommand(this::hasCoral);
    }
    public Command coralWaitShootCommand(){
        return new WaitUntilCommand(() -> !hasCoral());
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
