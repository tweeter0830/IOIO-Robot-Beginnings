/**
 *
 */
package tweeter0830.motorLibrary.TB661;
import tweeter0830.motorLibrary.DcMotor;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;

/**
 * @author Jacob Huffman
 *  This class is used to initialize and run one motor attached to a TB6612FNG motor driver
 *  It has the ability to initialize, run, brake, standby and power one motor. However,
 *  it doesn't take into account the fact that the TB6612FNG shares one standby wire
 *  between two motors. This class does not take that into account, so weird bugs may
 *  occur if two motors are attached on the same TB6612FNG and the standby function
 *  is used on one, but not two of the motors
 */
public class TB661Motor implements DcMotor{

	/**
	 *
	 */
	private DigitalOutput outputPin1_;
	private DigitalOutput outputPin2_;
	private PwmOutput  pwmPin_;
	private DigitalOutput standbyPin_;
	private IOIO ioio_;
	private boolean closed_ = false;

	private int pin1Setting_ = 0;
	private int pin2Setting_ = 0;
	private int pwmPinSetting_ = 0;
	private int standbyPinSetting_ = 0;

	//private ConnectionLostException exception_;
	
	public TB661Motor(int pin1Num, int pin2Num,
			int pwmPinNum, int standbyPinNum,
			int frequency, IOIO ioio) {
		if( closed_ )
			return;
		ioio_ = ioio;
		try{
			ioio_.softReset();
		}
		catch( ConnectionLostException e ){;
		}
//		try{
//			outputPin1_ = ioio_.openDigitalOutput(pin1Num);
//			outputPin2_ = ioio_.openDigitalOutput(pin2Num);
//			pwmPin_ =  ioio_.openPwmOutput(pwmPinNum, frequency);
//			standbyPin_ = ioio_.openDigitalOutput(standbyPinNum);
//		}
//		catch( ConnectionLostException e ){
//			exception_ = e;
//		}
	}

	@Override
	public void moveForward(double speed){
		if( closed_ )
			return;
		try{
			outputPin1_.write(true);
			outputPin2_.write(false);
			pwmPin_.setDutyCycle((float)constrainSpeed( speed ));
		}
		catch( ConnectionLostException e ){
		}
	}

	@Override
	public void moveBackward(double speed){
		if( closed_ )
			return;
		try{
			outputPin1_.write(false);
			outputPin2_.write(true);
			pwmPin_.setDutyCycle((float)constrainSpeed( speed ));
		}
		catch( ConnectionLostException e ){
		}
	}

	@Override
	public void move( double speed ){
		if( closed_ )
			return;
		if( speed < 0 )
			moveBackward( -1 * speed);
		else if( speed >= 0 )
			moveForward( speed );
	}

	@Override
	public void brake(){
		if( closed_ )
			return;
		try{
			outputPin1_.write(true);
			outputPin2_.write(true);
			pwmPin_.setDutyCycle(1);
		}
		catch( ConnectionLostException e ){
		}
		
	}

	@Override
	public void standby(){
		if( closed_ )
			return;
		try{
			standbyPin_.write(false);
		}
		catch( ConnectionLostException e ){
		}
	}

	@Override
	public void powerOn(){
		if( closed_ )
			return;
		try{
			standbyPin_.write(true);
		}
		catch( ConnectionLostException e ){
		}
	}

	public void close(){
		if( closed_ )
			return;
		outputPin1_.close();
		outputPin2_.close();
		standbyPin_.close();
		pwmPin_.close();
		closed_ =  true;
	}

	private double constrainSpeed( double speed ){
		if( speed < 0 )
			return 0.0;
		else
			return 1.0;
	}

}