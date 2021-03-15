import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MoveController;
import lejos.utility.Delay;

public class TravelTest {
	MoveController pilot;
	
	PoseProvider poseProvider;
	
	EV3UltrasonicSensor us = new EV3UltrasonicSensor(SensorPort.S4);
	SampleProvider bump = us.getDistanceMode();
	float[] sample = new float[1];
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		TravelTest traveler = new TravelTest();
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		traveler.pilot = new DifferentialPilot(4.3f, 12f, leftMotor, rightMotor);
		traveler.go();
	}

	public void go(){
		poseProvider = new OdometryPoseProvider(pilot);
		pilot.forward();
		SampleProvider sp  = us.getDistanceMode();
		
		while(pilot.isMoving()){
			sp.fetchSample(sample, 0);
			if(sample[0] <= 0.2){
				pilot.stop();
				Sound.twoBeeps();
			}
			LCD.drawString("X= " + poseProvider.getPose().getX(), 0, 1);
			LCD.drawString("Y= " + poseProvider.getPose().getY(), 0, 2);
		}
		
		LCD.drawString("Distance parcouru", 0, 1);
		double distance = pilot.getMovement().getDistanceTraveled();
		LCD.drawString("" + distance, 0, 2);
		
		Delay.msDelay(2000);
		//pilot.backward();
		pilot.travel(-distance);
		//pilot.travel(-1);
	}
	
}
