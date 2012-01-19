/**
 * 
 */
package tweeter0830.pidcontrol;

/**
 * @author Jacob Huffman
 *
 */
public class PID {
	//the factor to convert time to seconds
	//Example: seconds = nanoTime/10^9 Therefore TIMEINPUT = 1000000000L
	private static final long TIMEINPUT = 10000000000L;
	
	//constants
	double kp_;
	double ki_;
	double kd_;
	double filterTime_;
	double beta_;
	double gamma_;
	double windupFactor_;
	
	//These are updated every time updateInput is called
	//Controller coefficients 
	double integralWeight_;
	double oldDerivWeight_;
	double newDerivWeight_;
	double windupWeight_;
	
	//Persistent Variables
	long oldTime_;
	double oldProcessVar_;
	double oldDerivative_;
	double oldIntegral_;
	double satOutput_;

	
	//This is updated updateSetpoint
	double setpoint_;
	
	boolean firstStep = true;
	
	public PID(double kp, double ki, double kd){
		this(kp,ki,kd,99999,1,0,0);
	}
	
	public PID(double kp, double ki, double kd, double filterCoef){
		this(kp,ki,kd,filterCoef,1,0,0);
	}
	
	public PID(double kp, double ki, double kd, double filterCoef, double beta, double gamma, double windupFactor){
		kp_=kp;
		ki_=ki;
		kd_=kd;
		filterTime_ = (kd/kp)/filterCoef;
		beta_ = beta;
		gamma_ = gamma;
		windupFactor_ = windupFactor;
	}
	
	public void updateSetpoint(double setpoint){
		setpoint_ = setpoint;
	}
	
	//Update the PID with sensor information, but don't calculate the new output.
	//Persistent states (Derivative Value, Integral Value and old Time will
	//need to be updated
	public void updateProcessVar(double proccVar, long time){
		if( firstStep == true )
			return;
		
		
		long timeChange = time - oldTime_;
		computeCoef(timeChange/TIMEINPUT);
		double prop = kp_*(beta_*setpoint_-proccVar);
		oldDerivative_ = oldDerivWeight_*oldDerivative_-newDerivWeight_*(proccVar-oldProcessVar_);
		double unsatOutput = prop+oldDerivative_+oldIntegral_;
		satOutput_ = simulateOutputSat(unsatOutput);
		oldIntegral_ = oldIntegral_+integralWeight_*(setpoint_-proccVar)+
				windupWeight_*(satOutput_-unsatOutput);
		oldTime_ = time;
		oldProcessVar_=proccVar;
	}

	public double updateOutput(){
		if( firstStep == true )
			return 0;
		
		return satOutput_;
	}
	
	public void firstStep(double processVar, long time){
		oldProcessVar_ = processVar;
		oldTime_ = time;
		oldDerivative_ = 0;
		oldIntegral_ = 0;
		firstStep = false;
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
