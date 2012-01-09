package ioio.examples.simple;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.AbstractIOIOActivity;
import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
//Libraries for accessing the sensors
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.hardware.SensorEventListener;

public class IOIOSimpleApp extends AbstractIOIOActivity {
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
		private AnalogInput input_;
		private PwmOutput pwmOutput_;
		private DigitalOutput led_;
		private SensorManager sm_;
		private Sensor accel_;
		private AccelListener accelListener_;
		
		float accelX_;
		float accelY_;
		float accelZ_;
		
		long lastLoopTime_;
		long thisLoopTime_;
		double loopRate_;
		
		public void setup() throws ConnectionLostException {
			try {
				lastLoopTime_ = System.nanoTime();
				
				//Get the sensor manager object
				sm_ = (SensorManager) getSystemService(SENSOR_SERVICE);
				//Get a sensor object for the accelerometer
				accel_ = sm_.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
				accelListener_ = new AccelListener();
				sm_.registerListener(accelListener_, accel_, SensorManager.SENSOR_DELAY_FASTEST);
				
				input_ = ioio_.openAnalogInput(40);
				pwmOutput_ = ioio_.openPwmOutput(12, 100);
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
				
				setText("IOIO Setup :)");
				enableUi(true);
			} catch (ConnectionLostException e) {
				enableUi(false);
				throw e;
			}
		}
		
		public void loop() throws ConnectionLostException {
			try {
				updateLoopRate();
				
				final float reading = input_.read();
				//setText(Float.toString(reading));
				pwmOutput_.setPulseWidth(500 + seekBar_.getProgress() * 2);
				led_.write(!toggleButton_.isChecked());
				setText(Float.toString(accelX_), Double.toString(loopRate_));
				
				updateLoopTimes();
				sleep(5);
			} catch (InterruptedException e) {
				ioio_.disconnect();
			} catch (ConnectionLostException e) {
				enableUi(false);
				throw e;
			}
		}
		
		private void updateLoopRate(){
			thisLoopTime_ = System.nanoTime();
			loopRate_ = 1/( (double)(thisLoopTime_-lastLoopTime_)/10000000000L);
		}
		
		private void updateLoopTimes(){
			lastLoopTime_ = thisLoopTime_;
		}
		
		private class AccelListener implements SensorEventListener{
			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy){
			}

			@Override
			public void onSensorChanged(SensorEvent event) {
		        //synchronized (this) {
				accelX_ = event.values[0];
				accelY_ = event.values[1];
				accelZ_ = event.values[2];
			}
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