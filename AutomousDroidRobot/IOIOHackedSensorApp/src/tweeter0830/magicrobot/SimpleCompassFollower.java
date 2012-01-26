package tweeter0830.magicrobot;

//OLDTODO Figure out why the orientation being reported changes axis between screen turn on and off
	//I think that this was due to a failure in my app to unregister sensorListeners
//TODO find a concrete way to unregister my sensor listeners no matter what

//import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
//import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.AbstractIOIOActivity;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
import android.util.Log;
//Libraries for creating a motor driver class
import tweeter0830.pidcontrol.PIDController;

public class SimpleCompassFollower extends AbstractIOIOActivity {
	private TextView textView_;
	private TextView loopRateView_;
	private SeekBar seekBar_;
	private ToggleButton toggleButton_;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        textView_ = (TextView)findViewById(R.id.TextView);
        loopRateView_ = (TextView)findViewById(R.id.loopRateVal);
        seekBar_ = (SeekBar)findViewById(R.id.SeekBar);
        toggleButton_ = (ToggleButton)findViewById(R.id.ToggleButton);
        
        enableUi(false);
    }
    
	class IOIOMotoThread extends AbstractIOIOActivity.IOIOThread {
		private DigitalOutput led_;
		
		long lastLoopTime_;
		long thisLoopTime_;
		double loopRate_;
		PIDController pidController_;
		
		public void setup() throws ConnectionLostException {
			try {
				lastLoopTime_ = System.nanoTime();
				
				SensorManager sm = (SensorManager) getSystemService(SENSOR_SERVICE);
				
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
				
				pidController_ = new PIDController(sm);
				pidController_.setPID(1, .25, .5, 9999,1,99999);
				pidController_.setSetpoint(0);
				pidController_.setMotor(1, 10, 11, 3, 6, 100, ioio_ );
				pidController_.setMotor(2, 12, 13, 4, 6, 100, ioio_ );
				pidController_.powerOn();
				sleep(500);
				
				Log.d("Setup", "Got to the end of setup\n");
				enableUi(true);
			} catch (ConnectionLostException e) {
				enableUi(false);
				pidController_.close();
				throw e;
			} catch (InterruptedException e) {
				ioio_.disconnect();
				pidController_.close();
			}
		}
		
		public void loop() throws ConnectionLostException {
			try {
				boolean motorUpdated = pidController_.updateMotors(0);
				led_.write(false);
				setText(Boolean.toString(motorUpdated), Double.toString(pidController_.getProcessVar()));
				sleep(100);
			} catch (InterruptedException e) {
				ioio_.disconnect();
				pidController_.close();
			} catch (ConnectionLostException e) {
				enableUi(false);
				pidController_.close();
				throw e;
			} 
		}
		
		private void updateLoopRate(){
			thisLoopTime_ = System.nanoTime();
			loopRate_ = 1/( (double)(thisLoopTime_-lastLoopTime_)/1000000000L);
		}
		
		private void updateLoopTimes(){
			lastLoopTime_ = thisLoopTime_;
		}
	}

	@Override
	protected AbstractIOIOActivity.IOIOThread createIOIOThread() {
		return new IOIOMotoThread();
	}

	private void enableUi(final boolean enable) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				seekBar_.setEnabled(enable);
				toggleButton_.setEnabled(enable);
			}
		});
	}
	
	private void setText(final String str1) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				textView_.setText(str1);
			}
		});
	}
	
	private void setText(final String str1, final String str2) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				textView_.setText(str1);
				loopRateView_.setText(str2);
			}
		});
	}
	
} 