package tweeter0830.magicrobot;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.android.AbstractIOIOActivity;
import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
import tweeter0830.TB661Driver.TB661Driver;

//It looks like the motors saturate at about .1 to .15 duty cycle 
//TODO Figure out why the application has a couple of false starts before it actually begins
public class MotorCalibrate extends AbstractIOIOActivity {
	private TextView textView_;
	private SeekBar motor1SeekBar_;
	private SeekBar motor2SeekBar_;
	private ToggleButton motor1TogButton_;
	private ToggleButton motor2TogButton_;
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        textView_ = (TextView)findViewById(R.id.TextView);
        motor1SeekBar_ = (SeekBar)findViewById(R.id.MotorSlide1);
        motor2SeekBar_ = (SeekBar)findViewById(R.id.MotorSlide2);
        motor1TogButton_ = (ToggleButton)findViewById(R.id.MotorButton1);
        motor2TogButton_ = (ToggleButton)findViewById(R.id.MotorButton2);

        enableUi(false);
    }
	
	class IOIOThread extends AbstractIOIOActivity.IOIOThread {
		private DigitalOutput led_;
		private TB661Driver motorController_ = new TB661Driver();
		
		public void setup() throws ConnectionLostException {
			try {
				motorController_.setMotor(1, 10, 11, 3, 6, 100, ioio_ );
				motorController_.setMotor(2, 12, 13, 4, 6, 100, ioio_ );
				motorController_.powerOn();
				//Break both motors
				motorController_.brake(3);
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, false);
				setText("Reached end of Setup");
				enableUi(true);
			} catch (ConnectionLostException e) {
				enableUi(false);
				motorController_.close();
				throw e;
			}
		}
		
		public void loop() throws ConnectionLostException {
			try {
				double PWM1 = (double)motor1SeekBar_.getProgress()/500-1;
				double PWM2 = (double)motor2SeekBar_.getProgress()/500-1;
				setText(Double.toString(PWM1));
				if(motor1TogButton_.isChecked()){
					motorController_.move(1, PWM1);
				}else{
					motorController_.brake(1);
				}
				
				if(motor2TogButton_.isChecked()){
					motorController_.move(2, PWM2);
				}else{
					motorController_.brake(2);
				}
				sleep(10);
			} catch (InterruptedException e) {
				ioio_.disconnect();
				motorController_.close();
			} catch (ConnectionLostException e) {
				enableUi(false);
				motorController_.close();
				throw e;
			}
		}
	}

	@Override
	protected AbstractIOIOActivity.IOIOThread createIOIOThread() {
		return new IOIOThread();
	}

	private void enableUi(final boolean enable) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				motor1SeekBar_.setEnabled(enable);
				motor2SeekBar_.setEnabled(enable);
				motor1TogButton_.setEnabled(enable);
				motor2TogButton_.setEnabled(enable);
			}
		});
	}
	private void setText(final String str) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				textView_.setText(str);
			}
		});
	}
}