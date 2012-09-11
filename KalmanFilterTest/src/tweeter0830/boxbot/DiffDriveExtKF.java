package tweeter0830.boxbot;

import Jama.*
import java.math.*;
import android.util.Log;

public class DiffDriveExtKF{
   private static String LOGTAG_ = "DiffDriveExtKF";
   private final int nStates = 6;
   private final int nSensors = 7;
   
   //States
   private double[] X_ = new double[nStates];
   // Kinematics Jacobian
   private double[][] F_ = new double[nStates][nStates];
   // Process noise
   public double[][] Q_ = new double[nStates][nStates];
   // Covariance Estimate
   private double[][] P_ = new double[nStates][]nStates];
   // Sensor values 
   private double[] Z_ = new double[nSensors];
   // Sensor Jacobian
   private double[][] H_ = new double[nSensors][nStates];
   // Sensor noise
   public double[][] R_ = new double[nSensors][nStates];
   
   //Variables to keep track of initialization
   private boolean QSet_ = false;
   private boolean RSet_ = false;
   private boolean timeSet_ = false;
   
   //Constants
   private double earthRad_;
   private double wheelDiam_;
   private double wheelBase_;
   private double iniLatit_;
   private long oldTime_;
   private long newTime_;

   public DiffDriveExtKF(double wheelDiam, double wheelBase, double earthRad, double inititalLat){
	   wheelDiam_ = wheelDiam;
	   wheelBase_ = wheelBase;
	   earthRad_ = earthRad;
	   inititalLat_ = inititalLat;
   }
   
   public void initialize(){
	   oldTime_ = System.nanoTime();
	   timeSet_ = true;
   }
   
   public void updateGPS(double lat, double longi){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[6][1];
	   meas[0][0] = longi;
	   meas[1][0] = lat;
	   boolean[] measFlags = new boolean[6];
	   measFlags[0] = true;
	   measFlags[1] = true;
	   update(new SimpleMatrix(meas), measFlags);
	   //this.setState(new double[6]);
   }
   
   public void updateSpeeds(double leftSpeed, double rightSpeed){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[6][1];
	   meas[2][0] = leftSpeed;
	   meas[3][0] = rightSpeed;
	   boolean[] measFlags = new boolean[6];
	   measFlags[2] = true;
	   measFlags[3] = true;
	   update(new SimpleMatrix(meas), measFlags);
	   //this.setState(new double[6]);
   }
   
   public void updateAccel(double accel){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[6][1];
	   meas[4][0] = accel;
	   boolean[] measFlags = new boolean[6];
	   measFlags[4] = true;
	   update(new SimpleMatrix(meas), measFlags);
	   //this.setState(new double[6]);
   }
   
   public void updateHeading(double heading){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[6][1];
	   meas[5][0] = heading;
	   boolean[] measFlags = new boolean[6];
	   measFlags[5] = true;
	   update(new SimpleMatrix(meas), measFlags);
	   //this.setState(new double[6]);
   }
   
   public void getState(double[] outState){
	   for(int row = 0; row<x.numRows(); row++){
		   outState[row]=x.get(row);
	   }
   }
   
   public void getSimpleP(double[] outState){
	   SimpleMatrix diag = P.extractDiag();
	   for(int row = 0; row<diag.numRows(); row++){
		   outState[row]=diag.get(row);
	   }
   }
   
	protected void calcf(final double[] XIn, double[] XOut, double deltaT){
		double x = Xin[0];
		double y = Xin[1];
		double v = Xin[2];
		double a = Xin[3];
		double theta = Xin[4];
		double thetaDot = Xin[5];
		
		Xout[0] = x-(v+a*deltaT)*deltaT*Math.sin(thetaDot*deltaT+theta);
		Xout[1] = y+(v+a*deltaT)*deltaT*Math.cos(thetaDot*deltaT+theta);
		Xout[2] = v+a*deltaT;
		Xout[3] = a
		Xout[4] = theta+a*deltaT
		Xout[5] = thetaDot
	}
   
   protected void calcF(final SimpleMatrix inState, SimpleMatrix F,double timeChange){
       double easting = inState.get(0);
       double northing = inState.get(1);
       double velocity = inState.get(2);
       double accel = inState.get(3);
       double heading = inState.get(4);
       double headingDot = inState.get(5);

       F.setRow(0, 0,                  easting, 0, Math.sin(heading)*timeChange,0,velocity*Math.cos(heading)*timeChange,0);
       F.setRow(1, 0,                  0, northing, Math.cos(heading)*timeChange,0,-velocity*Math.sin(heading)*timeChange,0);
       F.setRow(2, 0,                  0, 0, velocity, timeChange,0,0);
       F.setRow(3, 0,                  0, 0, 0, accel,0,0);
       F.setRow(4, 0,                  0, 0, 0, 0,heading,timeChange);
       F.setRow(5, 0,                  0, 0, 0, 0,0,headingDot);
   }

	protected void calch(final double[] Xin, double[] hOut){
		double x = Xin[0];
		double y = Xin[1];
		double v = Xin[2];
		double a = Xin[3];
		double theta = Xin[4];
		double thetaDot = Xin[5];
		
		Hout[0] = x
		Hout[1] = y
		Hout[2] = v
		Hout[3] = a
		Hout[4] = theta
		Hout[5] = thetaDot
		Hout[6] = thetaDot
	}
   
   protected void calcH(double[][] H){
		//TODO set H to zero
       H[0][0] = 1
       H[1][1] = 1
       H[2][2] = 1
       H[3][3] = 1
       H[4][4] = 1
       H[5][5] = 1
       H[6][5] = 1
   }
   
   private double getTimeChange(){
	   newTime_ = System.nanoTime();
	   double timeChangeSec = ((double)(newTime_-oldTime_))/1000000000;
	   oldTime_ = newTime_;
	   return timeChangeSec;
   }
   
   public void setDiagonal(double[][] inMatrix, double[] inDiag){
	   for(int row = 0; row<inDiag.length; row++){
		   inMatrix[row, row] = inDiag[row];
	   }
   }
}