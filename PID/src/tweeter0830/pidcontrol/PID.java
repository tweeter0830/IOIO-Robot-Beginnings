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
	private double filterTime_ = 99999;
	private double beta_ = 1;
	private double windupFactor_ = 99999;
	
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
	private double satOutput_;

	
	//This is updated in updateSetpoint
	private double setpoint_ = 0;
	
	private boolean firstStep_ = true;
	private boolean outputSet_ = false;
	
	public void setPID(double kp, double ki, double kd){
		this.setPID(kp,ki,kd,99999,1,99999);
	}
	
	public void setPID(double kp, double ki, double kd, double filterCoef){
		this.setPID(kp,ki,kd,filterCoef,1,99999);
	}
	
	public void setPID(double kp, double ki, double kd, double filterCoef, double beta, double windupFactor){
		kp_=kp;
		ki_=ki;
		kd_=kd;
		filterTime_ = kd/filterCoef; //(kd/kp)/filterCoef;
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
		double unsatOutput = prop+oldDerivative_+oldIntegral_;
		satOutput_ = simulateOutputSat(unsatOutput);
		oldIntegral_ = oldIntegral_+integralWeight_*(setpoint_-proccVar)+
				windupWeight_*(satOutput_-unsatOutput);
		oldTime_ = time;
		oldProcessVar_=proccVar;
		outputSet_ = true;
		Log.v("PID", "Kp:"+kp_+"\tki:"+ki_+"\tkd:"+kd_+"\n");
		Log.v("PID", "P:"+prop+"\tI:"+oldIntegral_+"\tD:"+oldDerivative_+"\n");
		Log.v("PID", "UnSatOut:"+unsatOutput+"\tSatOut:"+satOutput_+"\n");
	}
	
	public boolean outputIsSet(){
		return outputSet_;
	}
	
	public double updateOutput(){
		if( firstStep_ == true )
			return 0;
		
		return satOutput_;
	}
	
	public void firstStep(double processVar, long time){
		oldProcessVar_ = processVar;
		oldTime_ = time;
		oldDerivative_ = 0;
		oldIntegral_ = 0;
		firstStep_ = false;
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
		oldDerivWeight_ = filterTime_/(filterTime_+deltaTime);
		newDerivWeight_ = kd_/(filterTime_+deltaTime);
		windupWeight_ = deltaTime/windupFactor_;
	}
	
}
