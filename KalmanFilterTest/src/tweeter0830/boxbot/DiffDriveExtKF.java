package tweeter0830.boxbot;

import org.ejml.simple.SimpleMatrix;

import android.util.Log;
import java.math.*;

public class DiffDriveExtKF{

   private static String LOGTAG_ = "KalmanFilter";

   //Constants
   double R_;
   double L_;
   double D_;
   double inititalLon_;
   
   

   public void DiffDriveExtKF(double diameter, double length, double earthRad, double longitude){
	   D_ = diameter;
	   L_ = length;
	   R_ = earthRad;
	   inititalLon_ = longitude;
   }

   private void calcF(final SimpleMatrix inState, SimpleMatrix F,double timeChange){
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

   private void calcf(final SimpleMatrix inState, SimpleMatrix outState, double timeChange){
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

   private void calcH(final SimpleMatrix inState, SimpleMatrix H,final boolean[] MeasFlags){
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

   private void calch(final SimpleMatrix inState, SimpleMatrix h, boolean[] MeasFlags){
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
}