package tweeter0830.boxbot;

import tweeter0830.pidcontrol.PID;
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
		private PID leftMotorPID_;
		private PID rightMotorPID_;
		
		@Override
		public void setup() throws ConnectionLostException {
			try {
				enableUi(true);
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
				arduUart_ = ioio_.openUart(6, 7, 115200, Uart.Parity.NONE, Uart.StopBits.ONE);
				arduIn_ = arduUart_.getInputStream();
				arduOut_ = arduUart_.getOutputStream();
				arduConnect_ = new ArduConnect(arduIn_, arduOut_);
				leftMotorPID_ = new PID();
				rightMotorPID_ = new PID();
				leftMotorPID_.setPID(.2, 20, 0);
				rightMotorPID_.setPID(.2, 20, 0);
				leftMotorPID_.setLimits(-255, 255);
				rightMotorPID_.setLimits(-255, 255);
				arduConnect_.setMode(ArduConnect.Mode.PWM);
				leftMotorPID_.setSetpoint(0);
				rightMotorPID_.setSetpoint(0);
			} catch (IOException e){
				enableUi(false);
				e.printStackTrace();
			} catch (ConnectionLostException e) {
				enableUi(false);
				throw e;
			}
		}
		long leftEncoder, rightEncoder, loopTime;
		int leftPWM, rightPWM;
		
		@Override
		public void loop() throws ConnectionLostException {
			try {
				loopTime = System.nanoTime();
				//Log.v(LOGTAG_, "Started Loop @ "+loopTime);
				arduConnect_.updateEncoders();
				leftEncoder = arduConnect_.getLeftEncoder();
				Log.v(LOGTAG_, "Left Encoder: "+leftEncoder);
				rightEncoder = arduConnect_.getRightEncoder();
				leftMotorPID_.updateProcessVar(leftEncoder, loopTime);
				rightMotorPID_.updateProcessVar(rightEncoder, loopTime);
				//Oh dear god, the output doesn't match what the PID is outputting. May cause errors
				leftPWM = (int)leftMotorPID_.outputUpdate();
				rightPWM = (int)rightMotorPID_.outputUpdate();
				Log.v(LOGTAG_, "Left PWM: "+leftPWM);
				arduConnect_.setPWM(leftPWM, rightPWM);
				loopTime = System.nanoTime();
				//Log.v(LOGTAG_, "Ended Loop @ "+loopTime+"\n");
				led_.write(!toggleButton_.isChecked());
				sleep(1);
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