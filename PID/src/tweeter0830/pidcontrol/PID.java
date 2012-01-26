/**
 * 
 */
package tweeter0830.pidcontrol;
import android.util.Log;

/**
 * @author Jacob Huffman
 *
 */
public class PID {
	//the factor to convert time to seconds
	//Example: seconds = nanoTime/10^9 Therefore TIMEINPUT = 1000000000L
	private static final long TIMEINPUT = 10000000000L;
	
	//constants
	private double kp_ = 1;
	private double ki_ = 0;
	private double kd_ = 0;
	private double filterTime_ = 0;
	private double beta_ = 1;
	private double windupFactor_ = 0;
	
	//These are updated every time updateInput is called
	//Controller coefficients 
	private double integralWeight_;
	private double oldDerivWeight_;
	private double newDerivWeight_;
	private double windupWeight_;
	
	//Persistent Variables
	private long oldTime_;
	private double oldProcessVar_;
	private double oldDerivative_;
	private double oldIntegral_;
	private double curSatOutput_;
	private double curUnsatOutput_;
	private double oldSatOutput_;
	private double oldUnsatOutput_;
	
	//This is updated in updateSetpoint
	private double setpoint_ = 0;
	
	private boolean firstStep_ = true;
	private boolean outputSet_ = false;
	
	public void setPID(double kp, double ki, double kd){
		this.setPID(kp,ki,kd,0,1,0);
	}
	
	public void setPID(double kp, double ki, double kd, double filterCoef){
		this.setPID(kp,ki,kd,filterCoef,1,0);
	}
	
	public void setPID(double kp, double ki, double kd, double filterCoef, double beta, double windupFactor){
		kp_=kp;
		ki_=ki;
		kd_=kd;
		filterTime_ = filterCoef*kd/kp;
		beta_ = beta;
		windupFactor_ = windupFactor;
	}
	
	public void setSetpoint(double setpoint){
		setpoint_ = setpoint;
	}
	
	public double getProcessVar(){
		return oldProcessVar_;
	}
	
	public boolean isInitialized(){
		return !firstStep_;
	}
	//Update the PID with sensor information, but don't calculate the new output.
	//Persistent states (Derivative Value, Integral Value and old Time will
	//need to be updated
	public void updateProcessVar(double proccVar, long time){
		if( firstStep_ == true ){
			firstStep( proccVar, time);
			return;
		}
		
		long timeChange = time - oldTime_;
		computeCoef((double)timeChange/TIMEINPUT);
		double prop = kp_*(beta_*setpoint_-proccVar);
		oldDerivative_ = oldDerivWeight_*oldDerivative_-newDerivWeight_*(proccVar-oldProcessVar_);
		curUnsatOutput_ = prop+oldDerivative_+oldIntegral_;
		curSatOutput_ = simulateOutputSat(curUnsatOutput_);
		oldIntegral_ = oldIntegral_+integralWeight_*(setpoint_-proccVar)+
				windupWeight_*(oldSatOutput_-oldUnsatOutput_);
		oldTime_ = time;
		oldProcessVar_=proccVar;
		outputSet_ = true;
		Log.v("PID", "Kp:"+kp_+"\tki:"+ki_+"\tkd:"+kd_+"\n");
		Log.v("PID", "P:"+prop+"\tI:"+oldIntegral_+"\tD:"+oldDerivative_+"\n");
		Log.v("PID", "CurrentUnSatOut:"+curUnsatOutput_+"\tCurrentSatOut:"+curSatOutput_+"\n");
		Log.v("PID", "OldUnSatOut:"+oldUnsatOutput_+"\tOldSatOut:"+oldSatOutput_+"\n");
	}
	
	public boolean outputIsSet(){
		return outputSet_;
	}
	
	public double outputUpdate(){
		if( firstStep_ == true )
			return 0;
		
		oldSatOutput_ = curSatOutput_;
		oldUnsatOutput_ = curUnsatOutput_;
		return curSatOutput_;
	}
	
	public void firstStep(double processVar, long time){
		oldProcessVar_ = processVar;
		oldTime_ = time;
		oldDerivative_ = 0;
		oldIntegral_ = 0;
		firstStep_ = false;
	}
	
	public double getCurrentOutput(){
		if( firstStep_ == true )
			return 0;
		
		return curSatOutput_;
	}
	
	private double simulateOutputSat(double unsatOutput){
		if(unsatOutput>1)
			return 1;
		else if(unsatOutput<-1)
			return -1;
		else 
			return unsatOutput;
	}
	
	private void computeCoef(double deltaTime){
		integralWeight_ = ki_*deltaTime;
		if(filterTime_ <= 0){
			oldDerivWeight_ = 0;
			newDerivWeight_ = 1;
		}else{
			oldDerivWeight_ = filterTime_/(filterTime_+deltaTime);
			newDerivWeight_ = kd_/(filterTime_+deltaTime);
		}
		if(windupFactor_ <= 0)
			windupWeight_ = 0;
		else
			windupWeight_ = deltaTime/windupFactor_;
	}
	
}
