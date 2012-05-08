package tweeter0830.boxbot;

import org.ejml.simple.SimpleMatrix;

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

   public void setState(SimpleMatrix x, SimpleMatrix P) {
               this.x.set(x);
               this.P.set(P);
   }

   public void predict(double timeChange) {
       // x = F x
       calcf(x,a,timeChange);
       x.set(a);

       // P = F P F' + Q
       calcF(x,F,timeChange);
       P = F.mult(P).mult(F.transpose()).plus(Q);
   }

   public void update(SimpleMatrix z, SimpleMatrix R, final boolean[] MeasFlags) {
       // y = z - h(x)
       calch(z, a, MeasFlags);
       y = z.minus(a);

       // S = H P H' + R
       calcH(z, H, MeasFlags);
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

   protected abstract void calcF(final SimpleMatrix inState, SimpleMatrix F, double timeChange);

   protected abstract void calcf(final SimpleMatrix inState, SimpleMatrix outState, double timeChange);

   protected abstract void calcH(final SimpleMatrix inState, SimpleMatrix H, boolean[] MeasFlags);

   protected abstract void calch(final SimpleMatrix inState, SimpleMatrix h, boolean[] MeasFlags);
}