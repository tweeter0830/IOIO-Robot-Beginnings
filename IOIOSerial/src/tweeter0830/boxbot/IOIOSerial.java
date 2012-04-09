package tweeter0830.boxbot;

import tweeter0830.boxbot.ArduConnect;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import ioio.lib.android.AbstractIOIOActivity;
import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalInput;
import ioio.lib.api.DigitalInput.Spec;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;
import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
import android.util.Log;

public class IOIOSerial extends AbstractIOIOActivity {
	private TextView textView_;
	private SeekBar seekBar_;
	private ToggleButton toggleButton_;
	public static final String LOGTAG_ = "Compass Follow";

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        textView_ = (TextView)findViewById(R.id.TextView);
        seekBar_ = (SeekBar)findViewById(R.id.SeekBar);
        toggleButton_ = (ToggleButton)findViewById(R.id.ToggleButton);

        enableUi(false);
    }
	
	class IOIOThread extends AbstractIOIOActivity.IOIOThread {
		private DigitalOutput led_;
		
		private ArduConnect arduConnect_;
		private Uart arduUart_;
		private InputStream arduIn_;
		private OutputStream arduOut_;
		
		@Override
		public void setup() throws ConnectionLostException {
			try {
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
				
//				DigitalInput.Spec rxPin = new DigitalInput.Spec(6,DigitalInput.Spec.Mode.PULL_UP);
//				DigitalOutput.Spec txPin = new DigitalOutput.Spec(7,DigitalOutput.Spec.Mode.OPEN_DRAIN);
				enableUi(true);
				arduUart_ = ioio_.openUart(6, 7, 115200, Uart.Parity.NONE, Uart.StopBits.ONE);
				arduIn_ = arduUart_.getInputStream();
				arduOut_ = arduUart_.getOutputStream();
				arduConnect_ = new ArduConnect(arduIn_, arduOut_);
			} catch (ConnectionLostException e) {
				enableUi(false);
				throw e;
			}
		}
		
		@Override
		public void loop() throws ConnectionLostException {
			try {
				Log.v(LOGTAG_, "Start Process");
				Log.v(LOGTAG_, "Change Mode");
				arduConnect_.setMode(ArduConnect.Mode.SPEED);
				//arduConnect_.setMode(ArduConnect.Mode.PWM);
				//arduConnect_.updateEncoders();
				//arduConnect_.updateSpeed();
				arduConnect_.setSpeed(1.0, 1.0);
				arduConnect_.setGains(20, 30, 0);
				Log.v(LOGTAG_, "Process Finished");
				led_.write(!toggleButton_.isChecked());
				sleep(1000);
			} catch (InterruptedException e) {
				ioio_.disconnect();
			} catch (ConnectionLostException e) {
				enableUi(false);
				throw e;
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
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
				seekBar_.setEnabled(enable);
				toggleButton_.setEnabled(enable);
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