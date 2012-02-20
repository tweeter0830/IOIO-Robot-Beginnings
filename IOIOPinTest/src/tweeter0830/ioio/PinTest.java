package tweeter0830.ioio;

import ioio.lib.android.AbstractIOIOActivity;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.AnalogInput;
import ioio.lib.api.exception.ConnectionLostException;
import android.os.Bundle;
import android.widget.TextView;
import tweeter0830.ioio.control.NumberPicker;

/**
 * 
 */
public class PinTest extends AbstractIOIOActivity {
	private NumberPicker numPicker1_;
	private TextView num1PinText_;

	/**
	 * Called when the activity is first created. Here we normally initialize
	 * our GUI.
	 */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		numPicker1_ = (NumberPicker) findViewById(R.id.numberPicker1);
		num1PinText_ = (TextView) findViewById(R.id.textView2);
		enableUi(false);
	}

	/**
	 * This is the thread on which all the IOIO activity happens. It will be run
	 * every time the application is resumed and aborted when it is paused. The
	 * method setup() will be called right after a connection with the IOIO has
	 * been established (which might happen several times!). Then, loop() will
	 * be called repetitively until the IOIO gets disconnected.
	 */
	class IOIOThread extends AbstractIOIOActivity.IOIOThread {
		/** The on-board LED. */
		private DigitalOutput led_;
		private DigitalOutput pin1Output_;
		private AnalogInput analogInput1_;
		private int pin1Num_;
		private int oldPin1Num_ = 999;
		/**
		 * Called every time a connection with IOIO has been established.
		 * Typically used to open pins.
		 * 
		 * @throws ConnectionLostException
		 *             When IOIO connection is lost.
		 * 
		 * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#setup()
		 */
		@Override
		protected void setup() throws ConnectionLostException {
			led_ = ioio_.openDigitalOutput(0, false);
			analogInput1_ = ioio_.openAnalogInput(46);
			pin1Num_ = numPicker1_.getValue();
			pin1Output_ = ioio_.openDigitalOutput(1, false);
			enableUi(true);
		}

		/**
		 * Called repetitively while the IOIO is connected.
		 * 
		 * @throws ConnectionLostException
		 *             When IOIO connection is lost.
		 * 
		 * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#loop()
		 */
		@Override
		protected void loop() throws ConnectionLostException {
			pin1Num_ = numPicker1_.getValue();
			if( (pin1Num_!=oldPin1Num_) && (1<=pin1Num_ && pin1Num_<=45) )
			{
				pin1Output_.close();
				pin1Output_ = ioio_.openDigitalOutput(pin1Num_, false);
				oldPin1Num_= pin1Num_;
			}	
			try {
				setText(Float.toString(analogInput1_.getVoltage()));
				sleep(100);
			} catch (InterruptedException e) {
				enableUi(false);
			}
		}
	}

	/**
	 * A method to create our IOIO thread.
	 * 
	 * @see ioio.lib.util.AbstractIOIOActivity#createIOIOThread()
	 */
	@Override
	protected AbstractIOIOActivity.IOIOThread createIOIOThread() {
		return new IOIOThread();
	}
	
	private void enableUi(final boolean enable) {
		runOnUiThread(new Runnable() {
			//@Override
			public void run() {
				numPicker1_.setEnabled(enable);
			}
		});
	}
	private void setText(final String str) {
		runOnUiThread(new Runnable() {
			//@Override
			public void run() {
				num1PinText_.setText(str);
			}
		});
	}
}