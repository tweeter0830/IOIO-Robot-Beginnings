package tweeter0830.boxbot;

import org.ejml.simple.SimpleMatrix;

import android.util.Log;
import java.math.*;

public class DiffDriveExtKF extends ExtendedKF{

   private static String LOGTAG_ = "KalmanFilter";

   //Constants
   private double R_;
   private double L_;
   private double D_;
   private double inititalLon_;
   private long oldTime_;
   private long newTime_;

   public DiffDriveExtKF(double diameter, double length, double earthRad, double longitude){
	   D_ = diameter;
	   L_ = length;
	   R_ = earthRad;
	   inititalLon_ = longitude;
	   x = new SimpleMatrix(6, 1);
	   P = new SimpleMatrix(6, 6);
	   
	   Q = new SimpleMatrix(6,6);
	   R = new SimpleMatrix(6,6);
   }
   
   public void initialize(){
	   oldTime_ = System.nanoTime();
   }
   
   public void setState(final double[] inState){
	   for(int row = 1; row<=inState.length; row++){
		   x.set(row,inState[row-1]);
	   }
   }
   
   public void updateGPS(double lat, double longi){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[7][1];
	   meas[1][1] = longi;
	   meas[2][1] = lat;
	   boolean[] measFlags = new boolean[6];
	   measFlags[0] = true;
	   measFlags[1] = true;
	   update(new SimpleMatrix(meas), measFlags);
   }
   
   public void updateSpeeds(double leftSpeed, double rightSpeed){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[7][1];
	   meas[3][1] = leftSpeed;
	   meas[4][1] = rightSpeed;
	   boolean[] measFlags = new boolean[6];
	   measFlags[2] = true;
	   measFlags[3] = true;
	   update(new SimpleMatrix(meas), measFlags);
   }
   
   public void updateAccel(double accel){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[7][1];
	   meas[5][1] = accel;
	   boolean[] measFlags = new boolean[6];
	   measFlags[4] = true;
	   update(new SimpleMatrix(meas), measFlags);
   }
   
   public void updateHeading(double heading){
	   predict(getTimeChange());
	   
	   double[][] meas = new double[7][1];
	   meas[6][1] = heading;
	   boolean[] measFlags = new boolean[6];
	   measFlags[5] = true;
	   update(new SimpleMatrix(meas), measFlags);
   }
   
   public void setSimpleP(final double[] inError){
	   setDiagonal(P, inError);
   }
   
   public void setSimpleQ(final double[] inError){
	   setDiagonal(Q, inError);
   }
   
   public void setSimpleR(final double[] inError){
	   setDiagonal(R, inError);
   }
   
   public void getState(double[] outState){
	   for(int row = 1; row<=x.numRows(); row++){
		   outState[row-1]=x.get(row);
	   }
   }
   
   public void getSimpleP(double[] outState){
	   SimpleMatrix diag = P.extractDiag();
	   for(int row = 1; row<=diag.numRows(); row++){
		   outState[row-1]=diag.get(row);
	   }
   }
   
   protected void calcF(final SimpleMatrix inState, SimpleMatrix F,double timeChange){
       double easting = inState.get(1);
       double northing = inState.get(2);
       double velocity = inState.get(3);
       double accel = inState.get(4);
       double heading = inState.get(5);
       double headingDot = inState.get(6);

       F.setRow(1, 1,                  easting, 0, Math.sin(heading)*timeChange,0,velocity*Math.cos(heading)*timeChange,0);
       F.setRow(2, 1,                  0, northing, Math.cos(heading)*timeChange,0,-velocity*Math.sin(heading)*timeChange,0);
       F.setRow(3, 1,                  0, 0, velocity, timeChange,0,0);
       F.setRow(4, 1,                  0, 0, 0, accel,0,0);
       F.setRow(5, 1,                  0, 0, 0, 0,heading,timeChange);
       F.setRow(6, 1,                  0, 0, 0, 0,0,headingDot);
   }

   protected void calcf(final SimpleMatrix inState, SimpleMatrix outState, double timeChange){
       double easting = inState.get(1);
       double northing = inState.get(2);
       double velocity = inState.get(3);
       double accel = inState.get(4);
       double heading = inState.get(5);
       double headingDot = inState.get(6);

       outState.set(1,velocity*Math.sin(heading)*timeChange+easting);
       outState.set(2,velocity*Math.cos(heading)*timeChange+northing);
       outState.set(3,accel*timeChange+velocity);
       outState.set(4,accel);
       outState.set(5,headingDot*timeChange+heading);
       outState.set(6, headingDot);
   }

   protected void calcH(final SimpleMatrix inState, SimpleMatrix H){
//      double easting = inState.get(1);
//      double northing = inState.get(2);
//      double velocity = inState.get(3);
//      double accel = inState.get(4);
//      double heading = inState.get(5);
//      double headingDot = inState.get(6);

       H.setRow(1, 1,                  Math.cos(inititalLon_),0,0,0,0,0);
       H.setRow(2, 1,                  0,1.0/R_,0,0,0,0);
       H.setRow(3, 1,                  0,0,1.0/D_,0,0,L_/(2*D_));
       H.setRow(4, 1,                  0,0,1.0/D_,0,0,-L_/(2*D_));
       H.setRow(5, 1,                  0,0,0,1,0,0);
       H.setRow(6, 1,                  0,0,0,0,1,0);
   }

   protected void calch(final SimpleMatrix inState, SimpleMatrix h){
       double easting = inState.get(1);
       double northing = inState.get(2);
       double velocity = inState.get(3);
       double accel = inState.get(4);
       double heading = inState.get(5);
       double headingDot = inState.get(6);

       //Change in Lon
       h.set(1,easting/R_*Math.cos(inititalLon_));
       //Change in Lat
       h.set(2,northing/R_);
       //Left Wheel Speed
       h.set(3,velocity/D_+headingDot*L_/(2*D_));
       //Right Wheel Speed
       //TODO This minus might need to be switched. Check defined direction of theta
       h.set(4,velocity/D_-headingDot*L_/(2*D_));
       //Acceleration
       h.set(5,accel);
       //Heading
       h.set(6,heading);
   }
   
   private double getTimeChange(){
	   long newTime = System.nanoTime();
	   double timeChangeSec = ((double)(newTime-oldTime_))/1000000000;
	   oldTime_ = newTime;
	   return timeChangeSec;
   }
   
   private void setDiagonal(SimpleMatrix inMatrix, double[] inDiag){
	   for(int row = 1; row<=inDiag.length; row++){
		   inMatrix.set(row, row, inDiag[row-1]);
	   }
   }
}