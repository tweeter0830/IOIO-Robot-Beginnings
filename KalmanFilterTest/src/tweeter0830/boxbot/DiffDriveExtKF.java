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
	   
	   Arrays.fill(Z_, 0);
	   //TODO Properly implement this. I need to map the Lat and long 
	   // to x and y in the movement frame
	   Z_[0] = lat;
	   Z_[1] = longi;
	   
	   boolean[] measFlags = new boolean[6];
	   measFlags[0] = true;
	   measFlags[1] = true;
	   
	   update(measFlags);
   }
   
   public void updateWheels(double leftRate, double rightRate){
	   predict(getTimeChange());
	   
	   Arrays.fill(Z_, 0);
	   Z_[2] = wheelDiam_*(leftRate+rightRate)/2;
	   Z_[7] = wheelDiam_*(rightRate-leftRate)/wheelBase_;
	   
	   boolean[] measFlags = new boolean[6];
	   measFlags[3] = true;
	   update(measFlags);
   }
   
   public void updateAccel(double accel){
	   predict(getTimeChange());
	   
	   Arrays.fill(Z_, 0);
	   Z_[3] = accel;
	   
	   boolean[] measFlags = new boolean[6];
	   measFlags[3] = true;
	   update(measFlags);
   }
   
   public void updateHeading(double heading){
	   predict(getTimeChange());
	   
	   Arrays.fill(Z_, 0);
	   Z_[4] = heading;
	   
	   boolean[] measFlags = new boolean[6];
	   measFlags[4] = true;
	   update(measFlags);
   }
   
   public void updateHeadingRate(double headingRate){
	   predict(getTimeChange());
	   
	   Arrays.fill(Z_, 0);
	   Z_[5] = headingRate;
	   
	   boolean[] measFlags = new boolean[6];
	   measFlags[5] = true;
	   update(measFlags);
   }
   
   public void predict(double deltaT) {
       // x = f(x)
	double[] XNew = new double[nStates];
       calcf(X_,XNew,deltaT);
       X_ = Arrays.copyOf(XNew,nStates);

       // P = F P F' + Q
       calcF(X_,F_,deltaT);
       
       //Matrix-ize everything :/
       Matrix FMat = Matrix(F_);
       Matrix PMat = Matrix(P_);
       Matrix QMat = Matrix(Q_);
       
       P_ = FMat.times(PMat).times(FMat.transpose()).plus(Q).getArrayCopy();
   }
   
   public void update(boolean[] MeasFlags) {
       
	/*** y = z - h(x) ***/
	double[] hCalced = new double[nSensors];
       calch(X_, hCalced);
       //Matrix-ize Everything :/
       Matrix ZMat = Matrix(Z_,nSensors);
       Matrix hMat = Matrix(hCalced,nSensors);
       
       Matrix YMat = ZMat.minus(hMat);
       
       /*** S = H P H' + R ***/
       calcH(X_, H_);
       wipeRows(H_, MeasFlags);
       
       //Matrix-ize Everything :/
       Matrix HMat = Matrix(Z_);
       Matrix PMat = Matrix(P_);
       Matrix RMat = Matrix(R_);
       
       Matrix SMat = HMat.times(PMat).times(HMat.transpose()).plus(RMat);
       
       /*** K = PH'S^(-1) ***/
       Matrix KMat = PMat.times(HMat.transpose()).times(SMat.invert());
       
       /*** x = x + Ky ***/
       X_ = XMat.plus(KMat.times(YMat)).getArrayCopy();

       /*** P = (I-kH)P = P - KHP ***/
       P_ = PMat.minus(KMat.mult(HMat).mult(PMat)).getArrayCopy();
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
   
   protected void calcF(final double[] XIn, double[][] F, double deltaT){
		double x = Xin[0];
		double y = Xin[1];
		double v = Xin[2];
		double a = Xin[3];
		double theta = Xin[4];
		double thetaDot = Xin[5];

		for (int line : 6) {
            Arrays.fill(F[line], 0);
        }
        
		double newTheta = thetaDot*deltaT+theta;
		
       F[0][0]=1; 
       F[0][2]=-deltaT*Math.sin(newTheta); 				F[0][3]=-deltaT^2*Math.sin(newTheta);
       F[0][4]=-(v+a*deltaT)*deltaT*Math.cos(newTheta);	F[0][5]=-(v+a*deltaT)*deltaT^2*Math.cos(newTheta);
       
       F[1][1]=1; 
       F[1][2]= deltaT*Math.cos(newTheta); 				F[1][3]= deltaT^2*Math.cos(newTheta);
       F[1][4]=-(v+a*deltaT)*deltaT*Math.sin(newTheta);	F[1][5]=-(v+a*deltaT)*deltaT^2*Math.sin(newTheta);
       
       F[2][2]=1;	F[2][3]=deltaT;
       F[3][3]=1;
       F[4][4]=1;	F[4][5]=deltaT;
       F[5][5]=1;
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
		for (int line : 6) {
            Arrays.fill(H[line], 0);
        }
        
       H[0][0] = 1
       H[1][1] = 1
       H[2][2] = 1
       H[3][3] = 1
       H[4][4] = 1
       H[5][5] = 1
       H[6][5] = 1
   }
   
   public void getState(double[] outState){
	   outState = Arrays.copyOf(X_,nStates);
   }
   
   public void setState(double[] inState){
	   X_ = Arrays.copyOf(outState,nStates);
   }
   
   public void getSimpleP(double[] POut){
	   for(int row = 0; row<nStates; row++){
		   POut[row]=P_[row][row];
	   }
   }
   
   private void wipeRows(double[][] inArray, final boolean[] MeasFlags){
	   for(int row = 0; row<nSensors; row++){
		   if( !MeasFlags[row]){
			   Arrays.fill(inArray[row],0);
		   }
	   }
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