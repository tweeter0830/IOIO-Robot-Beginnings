package tweeter0830.pidcontrol;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.IOIO;
import ioio.lib.api.exception.ConnectionLostException;
import java.util.Timer;
import java.util.TimerTask;

import android.util.Log;

public class Encoder {
	private static final String LOGTAG_ = "Encoder";
	private static final boolean isLogging_ = true;
	
	private IOIO ioio_;
	private boolean closed_ = false;
	private AnalogInput analogPin_ = null;
	private Timer timer_= new Timer();
	
	volatile private boolean direction_ = true;
	
	//this will be our count of how many times the encoders have switched values
	//this is not the same thing as the number of times the wheel has been turned
	//it needs to be converted for that
	private volatile long internalCount_ = 0;
	private volatile double speed_;
	
	public Encoder(IOIO ioio, int analogPinNum, double frequency) throws ConnectionLostException, InterruptedException{
		ioio_=ioio;
		analogPin_ = ioio_.openAnalogInput(analogPinNum);
		if(frequency>1000){
			if(isLogging_)
				Log.v(LOGTAG_, "Frequency is to high, setting to max (1000Hz)");
			frequency = 1000;
		}
		timer_.schedule(new EncoderTimer(), 0, (long)(1000/frequency));
	}
	
	public void setDirection(boolean direction){
		direction_ = direction;
	}
	public double getRotations(){
		return internalCount_/6.0;
	}
	public boolean getDirection(){
		return direction_;
	}
	public double getSpeed(){
		if(direction_ == true)
			return speed_/6.0;
		else
			return -speed_/6.0;
	}
	
	class EncoderTimer extends TimerTask{
		private boolean oldEncoderVal_;
		private long oldEncoderTime_ = 0;
		
		public EncoderTimer() throws InterruptedException, ConnectionLostException{
			oldEncoderVal_ = analogToBool(analogPin_.read());
		}
		public synchronized void run(){
			try {
				float inPinValue = analogPin_.read();
				boolean newEncoderVal = analogToBool(inPinValue);
				if(newEncoderVal==false && oldEncoderVal_==true && direction_ == true){
					internalCount_--;
				}
				else if(newEncoderVal==true && oldEncoderVal_==false && direction_ == false){
					internalCount_--;
				}
				if( newEncoderVal==false && oldEncoderVal_==true){
					internalCount_++;
					long newEncoderTime = System.nanoTime();
					if(oldEncoderTime_ == 0)
						speed_ = 0;
					else{
						//This speed is encoder tics per second. It gets changed into revolutions
						//per second on output
						speed_ = 1/( (double)(newEncoderTime-oldEncoderTime_)/10000000000L);
					}
					oldEncoderTime_ = newEncoderTime;
				}
				oldEncoderVal_=newEncoderVal;
				if(isLogging_)
					Log.v(LOGTAG_, "Count: " + internalCount_+ "\tSpeed: "+speed_+"\tPinValue: " +inPinValue);
			} catch (InterruptedException e) {
				this.cancel();
				e.printStackTrace();
			} catch (ConnectionLostException e) {
				this.cancel();
				e.printStackTrace();
			}
		}
		private boolean analogToBool(double in){
			if(in>=.5)
				return true;
			else
				return false;
		}
	}
}
