package tweeter0830.boxbot;

import static org.ejml.ops.CommonOps.*;
import org.ejml.simple.SimpleMatrix;

import android.util.Log;
import java.math.*;
/**
 * A Kalman filter is implemented by calling the generalized operations.  Much of the excessive
 * memory creation/destruction has been reduced from the KalmanFilterSimple.  However, there
 * is still room for improvement by using specialized algorithms directly.  The price paid
 * for this better performance is the need to manually manage memory and the need to have a
 * better understanding for how each of the operations works.
 *
 * @author Peter Abeles
 */
public class DiffDriveExtendedKF{
	
    // kinematics description
    private SimpleMatrix F;
    private SimpleMatrix Q;
    private SimpleMatrix H;

    // System state estimate
    private SimpleMatrix x;
    private SimpleMatrix P;

    // these are pre-declared for efficiency reasons
    private SimpleMatrix a,b;
    private SimpleMatrix y,S,S_inv,c,d;
    private SimpleMatrix K;

    private static String LOGTAG_ = "KalmanFilter";
    
    //Constants
    double R_;
    double L_;
    double D_;
    double inititalLon_;
//    public void configure(SimpleMatrix F, SimpleMatrix Q, SimpleMatrix H) {
//    	this.F = F;
//    	this.Q = Q;
//    	this.H = H;
//
//    	int dimenX = F.numCols;
//    	int dimenZ = H.numRows;
//
//    	a = new SimpleMatrix(dimenX,1);
//    	b = new SimpleMatrix(dimenX,dimenX);
//    	y = new SimpleMatrix(dimenZ,1);
//    	S = new SimpleMatrix(dimenZ,dimenZ);
//    	S_inv = new SimpleMatrix(dimenZ,dimenZ);
//    	c = new SimpleMatrix(dimenZ,dimenX);
//    	d = new SimpleMatrix(dimenX,dimenZ);
//    	K = new SimpleMatrix(dimenX,dimenZ);
//
//    	x = new SimpleMatrix(dimenX,1);
//    	P = new SimpleMatrix(dimenX,dimenX);
//    }

    public void setState(SimpleMatrix x, SimpleMatrix P) {
        if( x.numCols() == 1 && x.numRows() == 6 ){
        	this.x.set(x);
        	this.P.set(P);
        } else {
        	Log.e(LOGTAG_,"State or Covarience Input Matrix is not the correct size");
        }
    }

    public void predict(double timeChange) {

        // x = F x
    	predictState(x,a,timeChange);
    	x.set(a);

        // P = F P F' + Q
    	calcF(x,F,timeChange);
    	P = F.mult(P).mult(F.transpose()).plus(Q);
    }

    public void update(SimpleMatrix z, SimpleMatrix R) {

        // y = z - h(x)
    	calch(z, a);
        y = z.minus(a);

        // S = H P H' + R
        calcH(z, H);
        S = H.mult(P).mult(H.transpose()).plus(R);

        // K = PH'S^(-1)
        K = P.mult(H.transpose().mult(S.invert()));

        // x = x + Ky
        x = x.plus(K.mult(y));

        // P = (I-kH)P = P - KHP
        P = P.minus(K.mult(H).mult(P));
    }

    public SimpleMatrix getState() {
        return x;
    }

    public SimpleMatrix getCovariance() {
        return P;
    }
    private void calcF(final SimpleMatrix inState, SimpleMatrix F, double timeChange){
    	double easting = inState.get(1);
    	double northing = inState.get(2);
    	double velocity = inState.get(3);
    	double accel = inState.get(4);
    	double heading = inState.get(5);
    	double headingDot = inState.get(6);
    	
    	F.setRow(1, 1, 			easting, 0, Math.sin(heading)*timeChange, 0,velocity*Math.cos(heading)*timeChange,0);
    	F.setRow(2, 1, 			0, northing, Math.cos(heading)*timeChange, 0,-velocity*Math.sin(heading)*timeChange,0);
    	F.setRow(3, 1, 			0, 0, velocity, timeChange,0,0);
    	F.setRow(4, 1, 			0, 0, 0, accel,0,0);
    	F.setRow(5, 1, 			0, 0, 0, 0,heading,timeChange);
    	F.setRow(6, 1, 			0, 0, 0, 0,0,headingDot);
    }
    private void predictState(final SimpleMatrix inState, SimpleMatrix outState, double timeChange){
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
    private void calcH(final SimpleMatrix inState, SimpleMatrix H){
//    	double easting = inState.get(1);
//    	double northing = inState.get(2);
//    	double velocity = inState.get(3);
//    	double accel = inState.get(4);
//    	double heading = inState.get(5);
//    	double headingDot = inState.get(6);
    	
    	H.setRow(1, 1, 			Math.cos(inititalLon_),0,0,0,0,0);
    	H.setRow(2, 1, 			0,1.0/R_,0,0,0,0);
    	H.setRow(3, 1, 			0,0,1.0/D_,0,0,L_/(2*D_));
    	H.setRow(4, 1, 			0,0,1.0/D_,0,0,-L_/(2*D_));
    	H.setRow(5, 1, 			0,0,0,1,0,0);
    	H.setRow(6, 1, 			0,0,0,0,1,0);
    }
    private void calch(final SimpleMatrix inState, SimpleMatrix outh){
    	double easting = inState.get(1);
    	double northing = inState.get(2);
    	double velocity = inState.get(3);
    	double accel = inState.get(4);
    	double heading = inState.get(5);
    	double headingDot = inState.get(6);
    	
    	//Change in Lon
    	outh.set(1,easting/R_*Math.cos(inititalLon_));
    	//Change in Lat
    	outh.set(2,northing/R_);
    	//Left Wheel Speed
    	outh.set(3,velocity/D_+headingDot*L_/(2*D_));
    	//Right Wheel Speed
    	//TODO This minus might need to be switched. Check defined direction of theta
    	outh.set(4,velocity/D_-headingDot*L_/(2*D_));
    	//Acceleration
    	outh.set(5,accel);
    	//Heading
    	outh.set(6,heading);
    }
}