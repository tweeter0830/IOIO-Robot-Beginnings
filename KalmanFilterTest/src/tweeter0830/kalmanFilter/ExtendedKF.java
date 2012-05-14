package tweeter0830.kalmanFilter;

import org.ejml.simple.SimpleMatrix;

import android.util.Log;

/**
 * A Kalman filter is implemented by calling the generalized
operations.  Much of the excessive
 * memory creation/destruction has been reduced from the
KalmanFilterSimple.  However, there
 * is still room for improvement by using specialized algorithms
directly.  The price paid
 * for this better performance is the need to manually manage memory
and the need to have a
 * better understanding for how each of the operations works.
 *
 * @author Peter Abeles
 */
public abstract class ExtendedKF{
   private static String LOGTAG_ = "KalmanFilter";
	
   // kinematics description
   private SimpleMatrix F = new SimpleMatrix(6,6);
   protected SimpleMatrix Q;//Needs to be set
   private SimpleMatrix H = new SimpleMatrix(6,6);

   // System state estimate
   protected SimpleMatrix x;//Needs to be set
   protected SimpleMatrix P;//Needs to be set
   
   protected SimpleMatrix R;//Needs to be set

   // these are pre-declared for efficiency reasons
   private SimpleMatrix a = new SimpleMatrix(6,1),b;
   private SimpleMatrix y = new SimpleMatrix(6,6),S = new SimpleMatrix(6,6),S_inv,c,d;
   private SimpleMatrix K = new SimpleMatrix(6,6);

   public void predict(double timeChange) {
	   //Log.v(LOGTAG_, "Predict x in: "+x.get(0)+' '+x.get(1)+' '+x.get(2)+' '+x.get(3)+' '+x.get(4)+' '+x.get(5));
	   //Log.v(LOGTAG_, "Predict P in: "+P.get(0,0)+' '+P.get(1,1)+' '+P.get(2,2)+' '+P.get(3,3)+' '+P.get(4,4)+' '+P.get(5,5));
       // x = f(x)
	   a = new SimpleMatrix(6,1);
       calcf(x,a,timeChange);
       x.set(a);

       // P = F P F' + Q
       calcF(x,F,timeChange);
       P = F.mult(P).mult(F.transpose()).plus(Q);
       //Log.v(LOGTAG_, "Predict x out: "+x.get(0)+' '+x.get(1)+' '+x.get(2)+' '+x.get(3)+' '+x.get(4)+' '+x.get(5));
       //Log.v(LOGTAG_, "Predict P out: "+P.get(0,0)+' '+P.get(1,1)+' '+P.get(2,2)+' '+P.get(3,3)+' '+P.get(4,4)+' '+P.get(5,5));
   }

   public void update(SimpleMatrix z, boolean[] MeasFlags) {
	   //Log.v(LOGTAG_, "Update x in: "+x.get(0)+' '+x.get(1)+' '+x.get(2)+' '+x.get(3)+' '+x.get(4)+' '+x.get(5));
	   //Log.v(LOGTAG_, "Update P in: "+P.get(0,0)+' '+P.get(1,1)+' '+P.get(2,2)+' '+P.get(3,3)+' '+P.get(4,4)+' '+P.get(5,5));
	   Log.v(LOGTAG_, "Update z in: "+z.get(0)+' '+z.get(1)+' '+z.get(2)+' '+z.get(3)+' '+z.get(4)+' '+z.get(5));
       
	   /*** y = z - h(x) ***/
       calch(z, a);
       wipeRows(a, MeasFlags);
       y = z.minus(a);
       //Log.v(LOGTAG_, "Update y pre wipe: "+y.toString());
       //Log.v(LOGTAG_, "Update y post wipe: "+y.toString());
       
       /*** S = H P H' + R ***/
       calcH(z, H);
       Log.v(LOGTAG_, "Update H pre wipe: "+H.toString());
       wipeRows(H, MeasFlags);
       Log.v(LOGTAG_, "Update H post wipe: "+H.toString());
       S = (H.mult(P).mult(H.transpose())).plus(R);
       Log.v(LOGTAG_, "Update S "+S.toString());
       
       /*** K = PH'S^(-1) ***/
       K = P.mult(H.transpose().mult(S.invert()));
       a = S.invert();
       Log.v(LOGTAG_, "Update S-1 "+a.toString());
       //Log.v(LOGTAG_, "Update K "+K.toString());
       
       /*** x = x + Ky ***/
       x = x.plus(K.mult(y));

       /*** P = (I-kH)P = P - KHP ***/
       P = P.minus(K.mult(H).mult(P));
       //Log.v(LOGTAG_, "Update x out: "+x.get(0)+' '+x.get(1)+' '+x.get(2)+' '+x.get(3)+' '+x.get(4)+' '+x.get(5));
	   Log.v(LOGTAG_, "Update P out: "+P.get(0,0)+' '+P.get(1,1)+' '+P.get(2,2)+' '+P.get(3,3)+' '+P.get(4,4)+' '+P.get(5,5));
   }

   private void wipeRows(SimpleMatrix inMatrix, final boolean[] MeasFlags){
	   SimpleMatrix resizedMatrix = new SimpleMatrix(0, 0);
	   int rowCount = 0;
	   for(int row = 0; row<inMatrix.numRows(); row++){
			   if( MeasFlags[row]){
				   resizedMatrix.combine(rowCount, 0, inMatrix.extractVector(true, row));
			   }
	   }
   }
   
   private void wipeMatrix(SimpleMatrix inMatrix, final boolean[] MeasFlags){
	   SimpleMatrix resizedMatrix = new SimpleMatrix(0, 0);
	   int rowCount = 0;
	   for(int row = 0; row<inMatrix.numRows(); row++){
			   if( MeasFlags[row]){
				   resizedMatrix.combine(rowCount, 0, inMatrix.extractVector(true, row));
			   }
	   }
   }
//   public SimpleMatrix getState() {
//       return x;
//   }
//
//   public SimpleMatrix getCovariance() {
//       return P;
//   }

   protected abstract void calcF(final SimpleMatrix inState, SimpleMatrix F, double timeChange);

   protected abstract void calcf(final SimpleMatrix inState, SimpleMatrix outState, double timeChange);

   protected abstract void calcH(final SimpleMatrix inState, SimpleMatrix H);

   protected abstract void calch(final SimpleMatrix inState, SimpleMatrix h);
}