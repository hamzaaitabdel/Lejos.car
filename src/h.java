import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import javax.xml.stream.XMLStreamException;

import lejos.ev3.tools.PCNavigationModel;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.mapping.SVGMapLoader;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MoveController;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;

public class h {
	MoveController pilot;
	static SVGMapLoader mapLoader;
	PoseProvider poseProvider;
	static EV3LargeRegulatedMotor leftMotor;
	static EV3LargeRegulatedMotor rightMotor;
	//EV3UltrasonicSensor us = new EV3UltrasonicSensor(SensorPort.S4);
	//SampleProvider bump = us.getDistanceMode();
	float[] sample = new float[1];
	public static void main(String[] args) throws FileNotFoundException {
		// TODO Auto-generated method stub
		String text="<svg width=\"132.0\" height=\"340.0\" xmlns=\"http://www.w3.org/2000/svg\"><g><line stroke=\"#000000\" x1=\"32.0\" x2=\"32.0\" y1=\"0.0\" y2=\"88.0\"/><line stroke=\"#000000\" x1=\"32.0\" x2=\"0.0\" y1=\"88.0\" y2=\"88.0\"/><line stroke=\"#000000\" x1=\"0.0\" x2=\"0.0\" y1=\"88.0\" y2=\"340.0\"/><line stroke=\"#000000\" x1=\"0.0\" x2=\"95.0\" y1=\"340.0\" y2=\"340.0\"/><line stroke=\"#000000\" x1=\"95.0\" x2=\"95.0\" y1=\"340.0\" y2=\"294.0\"/><line stroke=\"#000000\" x1=\"95.0\" x2=\"132.0\" y1=\"294.0\" y2=\"294.0\"/><line stroke=\"#000000\" x1=\"132.0\" x2=\"132.0\" y1=\"294.0\" y2=\"0.0\"/><line stroke=\"#000000\" x1=\"132.0\" x2=\"32.0\" y1=\"0.0\" y2=\"0.0\"/></g></svg>";
		h traveler = new h();
		File f= new File("draw.svg");
		//System.out.println(f.getAbsolutePath());
//		FileWriter fw;
//		System.out.println("1");
//		try {
//			fw = new FileWriter("draw.svg");
//			fw.write(text);
//			System.out.println("d");
//			fw.close();
//		} catch (IOException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//			System.out.println("3");
//		}
		//System.out.println(f.exists());
		Delay.msDelay(10000);
		InputStream inputStream =new DataInputStream(new FileInputStream(f));
		mapLoader=new SVGMapLoader(inputStream);
		LineMap lm=null;
		try {
			lm=mapLoader.readLineMap();
		} catch (XMLStreamException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
		traveler.pilot = new DifferentialPilot(4.3f, 12f, leftMotor, rightMotor);
		
		traveler.go(lm);
	}

	public void go(LineMap lm){
		poseProvider = new OdometryPoseProvider(pilot);
		int j=0;
		
			
		pilot.setLinearSpeed(1000);
		 while(true){
			 LCD.drawString("X= " + poseProvider.getPose().getX(), 0, 1);
			 LCD.drawString("Y= " + poseProvider.getPose().getY(), 0, 2);
			
			 LCD.drawString(lm.inside(new Point(poseProvider.getPose().getX(),poseProvider.getPose().getY()))+"", 0, 3);
			 if(!lm.inside(new Point(poseProvider.getPose().getX(),poseProvider.getPose().getY()))){
				 pilot.stop();
				 Delay.msDelay(1000);
				 pilot.travel(-14);
				 leftMotor.rotate(120);
				 pilot.forward();
			 }
			 
//				SampleProvider sp  = us.getDistanceMode();
//				while(pilot.isMoving()){
//					sp.fetchSample(sample, 0);
//					if(Button.ESCAPE.isUp())break;
//					if(sample[0] <= 0.2){
//						pilot.stop();
//						Sound.twoBeeps();
//					}
//					LCD.drawString("X= " + poseProvider.getPose().getX(), 0, 1);
//					LCD.drawString("Y= " + poseProvider.getPose().getY(), 0, 2);
//				}
				//pilot.travel(-4);
//                leftMotor.forward();
//                Delay.msDelay(700);
//                leftMotor.stop();
                //pilot.forward();
				//LCD.drawString("Distance parcouru", 0, 3);
				//double distance = pilot.getMovement().getDistanceTraveled();
				//LCD.drawString("" + distance, 0, 2);
				
				//Delay.msDelay(1000);
				//pilot.backward();
				//pilot.travel(-distance);
				//pilot.travel(-1);
		 }
		
	}
	
}
