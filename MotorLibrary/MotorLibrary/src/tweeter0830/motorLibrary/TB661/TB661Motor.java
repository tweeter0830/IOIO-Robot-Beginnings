/**
 * 
 */
package tweeter0830.motorLibrary.TB661;
import tweeter0830.motorLibrary.DcMotor;
/**
 * @author jacob
 *
 */
public abstract class TB661Motor implements DcMotor{

	/**
	 * 
	 */
	private int pin1Val_;
	private int pin2Val_;
	private int pwmPinVal_;
	private int standbyPinVal_;
	
	public TB661Motor(int pin1Val, int pin2Val, int pwmPinVal, int standbyPinVal) {
		pin1Val_ = pin1Val;
		pin2Val_ = pin2Val;
		pwmPinVal_ = pwmPinVal;
		standbyPinVal_ = standbyPinVal;
		initializePin(pin1Val_);
		initializePin(pin2Val_);
		initializePin(pwmPinVal_);
		initializePin(standbyPinVal_);
	}

	public moveForward
	abstract void initializePin(int pinVal);
	abstract void freePin(int pinVal);
	abstract void setPin(int PinVal, boolean highLow);
	
}
