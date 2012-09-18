package tweeter0830.boxbot;

import Jama.*;
import java.util.Arrays;
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
	public double[][] P_ = new double[nStates][nStates];
	// Sensor values 
	private double[] Z_ = new double[nSensors];
	// Sensor Jacobian
	private double[][] H_ = new double[nSensors][nStates];
	// Sensor noise
	public double[][] R_ = new double[nSensors][nSensors];

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
		iniLatit_ = inititalLat;
	}

	public void initialize(){
		oldTime_ = System.nanoTime();
		timeSet_ = true;
	}
	
	public void updateAll(double[] measMatrix){
		predict(getTimeChange());

		Z_ = Arrays.copyOf(measMatrix, 7);
		boolean[] measFlags = new boolean[7];
		measFlags[0] = true;
		measFlags[1] = true;
		measFlags[2] = true;
		measFlags[3] = true;
		measFlags[4] = true;
		measFlags[5] = true;
		measFlags[6] = true;
		
		update(measFlags);
	}
	
	public void updateGPS(double lat, double longi){
		predict(getTimeChange());

		Arrays.fill(Z_, 0);
		//TODO Properly implement this. I need to map the Lat and long 
		// to x and y in the movement frame
		Z_[0] = lat;
		Z_[1] = longi;

		boolean[] measFlags = new boolean[7];
		measFlags[0] = true;
		measFlags[1] = true;

		update(measFlags);
	}

	public void updateWheels(double leftRate, double rightRate){
		predict(getTimeChange());

		Arrays.fill(Z_, 0);
		Z_[2] = wheelDiam_*(leftRate+rightRate)/2;
		Z_[7] = wheelDiam_*(rightRate-leftRate)/wheelBase_;

		boolean[] measFlags = new boolean[7];
		measFlags[3] = true;
		measFlags[6] = true;
		update(measFlags);
	}

	public void updateAccel(double accel){
		predict(getTimeChange());

		Arrays.fill(Z_, 0);
		Z_[3] = accel;

		boolean[] measFlags = new boolean[7];
		measFlags[3] = true;
		update(measFlags);
	}

	public void updateHeading(double heading){
		predict(getTimeChange());

		Arrays.fill(Z_, 0);
		Z_[4] = heading;

		boolean[] measFlags = new boolean[7];
		measFlags[4] = true;
		update(measFlags);
	}

	public void updateHeadingRate(double headingRate){
		predict(getTimeChange());

		Arrays.fill(Z_, 0);
		Z_[5] = headingRate;

		boolean[] measFlags = new boolean[7];
		measFlags[5] = true;
		update(measFlags);
	}

	public void predict(double deltaT) {
		// x = f(x)
		double[] XNew = new double[nStates];
		
//		Log.v(LOGTAG_, "Predict:\n XIn:" + Arrays.toString(X_));
		calcf(X_,XNew,deltaT);
		X_ = XNew;
//		Log.v(LOGTAG_, "XOut:" + Arrays.toString(X_));
		
		// P = F P F' + Q
		calcF(X_,F_,deltaT);

//		Log.v(LOGTAG_, "F_:" + Arrays.deepToString(F_));
		//Matrix-ize everything :/
		Matrix FMat = new Matrix(F_);
		Matrix PMat = new Matrix(P_);
		Matrix QMat = new Matrix(Q_);

		P_ = FMat.times(PMat).times(FMat.transpose()).plus(QMat).getArrayCopy();
//		Log.v(LOGTAG_, "P_:" + Arrays.deepToString(P_));
	}

	public void update(boolean[] MeasFlags) {

		/*** y = z - h(x) ***/
		double[] hCalced = new double[nSensors];
		calch(X_, hCalced);
//		Log.v(LOGTAG_, "Update\nXin:" + Arrays.toString(X_));
		//Matrix-ize Everything :/
		Matrix ZMat = new Matrix(Z_,nSensors);
		Matrix hMat = new Matrix(hCalced,nSensors);

		Matrix YMat = ZMat.minus(hMat);

//		Log.v(LOGTAG_, "YMat:" + Arrays.deepToString(YMat.getArrayCopy()));
		/*** S = H P H' + R ***/
		calcH(H_);
//		Log.v(LOGTAG_, "H_ PreWipe:" + Arrays.deepToString(H_));
//		Log.v(LOGTAG_, "MeasFlags:" + Arrays.toString(MeasFlags));
		//wipeRows(H_, MeasFlags);
//		Log.v(LOGTAG_, "H_ PostWipe:" + Arrays.deepToString(H_));
		//Matrix-ize Everything :/
		Matrix HMat = new Matrix(H_);
		Matrix PMat = new Matrix(P_);
		Matrix RMat = new Matrix(R_);
		Matrix XMat = new Matrix(X_,nStates);

		Matrix SMat = HMat.times(PMat).times(HMat.transpose()).plus(RMat);
//		Log.v(LOGTAG_, "S:" + Arrays.deepToString(SMat.getArray()));
		/*** K = PH'S^(-1) ***/
		Matrix KMat = PMat.times(HMat.transpose()).times(SMat.inverse());

		/*** x = x + Ky ***/
		X_ = XMat.plus(KMat.times(YMat)).getRowPackedCopy();

		/*** P = (I-kH)P = P - KHP ***/
		P_ = PMat.minus(KMat.times(HMat).times(PMat)).getArrayCopy();
	}

	protected void calcf(final double[] XIn, double[] XOut, double deltaT){
		double x = XIn[0];
		double y = XIn[1];
		double v = XIn[2];
		double a = XIn[3];
		double theta = XIn[4];
		double thetaDot = XIn[5];

		XOut[0] = x-(v+a*deltaT)*deltaT*Math.sin(thetaDot*deltaT+theta);
		XOut[1] = y+(v+a*deltaT)*deltaT*Math.cos(thetaDot*deltaT+theta);
		XOut[2] = v+a*deltaT;
		XOut[3] = a;
		XOut[4] = theta+a*deltaT;
		XOut[5] = thetaDot;
	}

	protected void calcF(final double[] XIn, double[][] F, double deltaT){
		double x = XIn[0];
		double y = XIn[1];
		double v = XIn[2];
		double a = XIn[3];
		double theta = XIn[4];
		double thetaDot = XIn[5];

		for (int line=0; line<nStates; line++) {
			Arrays.fill(F[line], 0);
		}

		double newTheta = thetaDot*deltaT+theta;

		F[0][0]=1; 
		F[0][2]=-deltaT*Math.sin(newTheta); 				F[0][3]=-deltaT*deltaT*Math.sin(newTheta);
		F[0][4]=-(v+a*deltaT)*deltaT*Math.cos(newTheta);	F[0][5]=-(v+a*deltaT)*deltaT*deltaT*Math.cos(newTheta);

		F[1][1]=1; 
		F[1][2]= deltaT*Math.cos(newTheta); 				F[1][3]= deltaT*deltaT*Math.cos(newTheta);
		F[1][4]=-(v+a*deltaT)*deltaT*Math.sin(newTheta);	F[1][5]=-(v+a*deltaT)*deltaT*deltaT*Math.sin(newTheta);

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

		hOut[0] = x;
		hOut[1] = y;
		hOut[2] = v;
		hOut[3] = a;
		hOut[4] = theta;
		hOut[5] = thetaDot;
		hOut[6] = thetaDot;
	}

	protected void calcH(double[][] H){
		for (int line=0;line<nSensors;line++) {
			Arrays.fill(H[line], 0);
		}

		H[0][0] = 1;
		H[1][1] = 1;
		H[2][2] = 1;
		H[3][3] = 1;
		H[4][4] = 1;
		H[5][5] = 1;
		H[6][5] = 1;
	}

	public void getState(double[] outState){
		for( int row = 0; row<nStates; row++)
			outState[row] = X_[row];
	}

	public void setState(double[] inState){
		for( int row = 0; row<nStates; row++)
			X_[row] = inState[row];
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

	private void wipeColumns(double[][] inArray, final boolean[] MeasFlags){
		for(int col = 0; col<nStates; col++){
			if( !MeasFlags[col]){
				Arrays.fill(inArray[col],0);
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
			inMatrix[row][row] = inDiag[row];
		}
	}
}