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
//Libraries for creating a motor driver class
import tweeter0830.motorLibrary.TB661.TB661Motor;

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
		private Sensor accelSensor_;
		private Sensor magSensor_;
		//private AccelListener accelListener_;
		
		private TB661Motor motor1_ = null;
		private TB661Motor motor2_;
		
		float[] accelVector_ = new float[3];
		float[] magVector_ = new float[3];
		
		long lastLoopTime_;
		long thisLoopTime_;
		double loopRate_;
		
		public void setup() throws ConnectionLostException {
			try {
				lastLoopTime_ = System.nanoTime();
				
				setText("IOIO Setup :)");
				
				//Get the sensor manager object
				sm_ = (SensorManager) getSystemService(SENSOR_SERVICE);
				//Get a sensor object for the accelerometer
				accelSensor_ = sm_.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
				magSensor_ = sm_.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
				AccelListener accelListener = new AccelListener();
				sm_.registerListener(accelListener, accelSensor_, SensorManager.SENSOR_DELAY_FASTEST);
				sm_.registerListener(accelListener, magSensor_, SensorManager.SENSOR_DELAY_FASTEST);
				
				input_ = ioio_.openAnalogInput(40);
				pwmOutput_ = ioio_.openPwmOutput(12, 100);
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
				IOIO ioio = ioio_;
				//DigitalOutput outputPin1 = ioio.openDigitalOutput(10);
				//outputPin1.write(true);
				//set up Motor
				setText("Got to Motor Setup");
				motor1_ = new TB661Motor(10, 11, 3, 6, 10, ioio_ );
				setText("Got past Motor Setup");
				motor1_.moveForward(1);
				setText("Moving");
//				if(thisLoopTime_ % 100000 == 0){
//					setText("trying to Move");
//					motor1_.move(0.5);
//					setText("Moved");
//				}
					
				
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
				
				float azOrientation = getAzOrientation(accelVector_, magVector_);
				
				final float reading = input_.read();
				//setText(Float.toString(reading));
				pwmOutput_.setPulseWidth(500 + seekBar_.getProgress() * 2);
				led_.write(!toggleButton_.isChecked());
				
				
				setText(Float.toString(azOrientation), Double.toString(loopRate_));
				
				updateLoopTimes();
				sleep(5);
			} catch (InterruptedException e) {
				ioio_.disconnect();
			} catch (ConnectionLostException e) {
				enableUi(false);
				throw e;
			}
		}
		
		private float getAzOrientation( float[] accelArray, float[] magArray){
			float[] rotMatrix = new float[9];
			float[] magIncMatrix = new float[9];
			float[] returnVals = new float[3];
			
			SensorManager.getRotationMatrix(rotMatrix, magIncMatrix, accelArray, magArray);
			returnVals = SensorManager.getOrientation(rotMatrix, magIncMatrix);
			
			return returnVals[0];
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
				if( event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
					accelVector_[0] = event.values[0];
					accelVector_[1] = event.values[1];
					accelVector_[2] = event.values[2];
				}
				else if(event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
				{
					magVector_[0] = event.values[0];
					magVector_[1] = event.values[1];
					magVector_[2] = event.values[2]; 
				}
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