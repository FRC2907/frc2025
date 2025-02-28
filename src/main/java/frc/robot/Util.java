package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Control;
import edu.wpi.first.wpilibj.PS5Controller;

public class Util {
    public static double arrayAverage(double arr[]){
        double total = 0;
        for (int i = 0; i < arr.length; i++){
            total += arr[i];
        }
        return total / arr.length;
    }
    public static double[] arrayReplace(double arr[], int position, double input){
        arr[position] = input;
        return arr;
    }

    public static double clamp(double min, double value, double max) {
        if (max < min) { 
            System.err.println("[EE] I was asked to clamp value " + value + " between min " + min + " and max " + max);
            new Exception().printStackTrace();
        }
        if (value < min) return min;
        if (max < value) return max;
        return value;
    }

    public static boolean checkDriverDeadband(PS5Controller input){
        return Math.abs(input.getLeftY()) >  Control.kDriverDeadband
            || Math.abs(input.getLeftX()) >  Control.kDriverDeadband
            || Math.abs(input.getRightY()) > Control.kDriverDeadband
            || Math.abs(input.getRightX()) > Control.kDriverDeadband;
    }
    public static boolean checkPOVUp(PS5Controller input){
        return input.getPOV() < Control.kPOVDeadband && input.getPOV() > 360 - Control.kPOVDeadband;
    }
    public static boolean checkPOVDown(PS5Controller input){
        return input.getPOV() < Control.kPOVDeadband + 180 && input.getPOV() > 180 - Control.kPOVDeadband;
    }
    public static double getLeftMagnitude(PS5Controller input){
        return Math.sqrt(Math.pow(input.getLeftX(), 2.0) + Math.pow(input.getLeftY(), 2.0));
    }
    public static double getRightMagnitude(PS5Controller input){
        return Math.sqrt(Math.pow(input.getRightX(), 2.0) + Math.pow(input.getRightY(), 2.0));
    }

    public static boolean isBlue(){
		if (DriverStation.getAlliance().isPresent()){
      	    return DriverStation.getAlliance().get() == Alliance.Blue;
		}
        System.err.println("[EE] I could not find an alliance");
        return true;
	}
    public static boolean isRed(){
		if (DriverStation.getAlliance().isPresent()){
      	    return DriverStation.getAlliance().get() == Alliance.Red;
		}
        System.err.println("[EE] I could not find an alliance");
		return true;
	}

    public static double metersPerSecondToRPM(double speed, double diameter){
        return (speed / (diameter * Math.PI)) * 60;
    }
    public static double revolutionsToMeters(double revolutions, double diameter){
        return revolutions * diameter * Math.PI;
    }
    public static double RPMToMetersPerSecond(double rpm, double diameter){
        return rpm * diameter * Math.PI / 60;
    }
}