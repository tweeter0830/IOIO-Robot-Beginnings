package tweeter0830.magicrobot;

//OLDTODO Figure out why the orientation being reported changes axis between screen turn on and off
	//I think that this was due to a failure in my app to unregister sensorListeners
//TODO find a concrete way to unregister my sensor listeners no matter what
//TODO add a button that when pressed will move the robot to an unpredictable (or set) theta
//TODO look up how to make a class multithread safe

//import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
//import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.android.AbstractIOIOActivity;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
import android.util.Log;
//Libraries for creating a motor driver class
import tweeter0830.pidcontrol.PIDController;

public class SimpleCompassFollower extends AbstractIOIOActivity {
	private TextView PIDOutputView_;
	private EditText setpointView_;
	private EditText kpView_;
	private EditText kiView_;
	private EditText kdView_;
	private EditText betaView_;
	private EditText filterView_;
	private EditText windupView_;
	private ToggleButton proccesingButton_;
	private Button resetButton_;


	private double defaultkp_;
	private double defaultki_;
	private double defaultkd_;
	private double defaultBeta_;
	private double defaultFilter_;
	private double defaultWindup_;
	private double defaultSetpoint_;
			
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        PIDOutputView_ = 	(TextView)findViewById(R.id.outputView);
        setpointView_ = 	(EditText)findViewById(R.id.editTextSetpoint);
        kpView_ = 			(EditText)findViewById(R.id.editTextKp);
        kiView_ = 			(EditText)findViewById(R.id.editTextKi);
        kdView_ = 			(EditText)findViewById(R.id.editTextKd);
        betaView_ = 		(EditText)findViewById(R.id.editTextBeta);
        filterView_ = 		(EditText)findViewById(R.id.editTextFilter);
        windupView_ = 		(EditText)findViewById(R.id.editTextWindup);
        proccesingButton_ = (ToggleButton)findViewById(R.id.ProccesToggle);
        resetButton_ = 		(Button)findViewById(R.id.resetButton);
        
    	defaultkp_ = Double.valueOf(this.getString(R.string.defaultKpVal).trim()).doubleValue();
    	defaultki_ = Double.valueOf(this.getString(R.string.defaultKiVal).trim()).doubleValue();
    	defaultkd_ = Double.valueOf(this.getString(R.string.defaultKdVal).trim()).doubleValue();
    	defaultBeta_ = Double.valueOf(this.getString(R.string.defaultBetaVal).trim()).doubleValue();
    	defaultFilter_ = Double.valueOf(this.getString(R.string.defaultFilterVal).trim()).doubleValue();
    	defaultWindup_ = Double.valueOf(this.getString(R.string.defaultWindupVal).trim()).doubleValue();
    	defaultSetpoint_ = Double.valueOf(this.getString(R.string.defaultSetpointVal).trim()).doubleValue();
    	
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
				pidController_.setPID(	defaultkp_,
										defaultki_, 
										defaultkd_, 
										defaultFilter_,
										defaultBeta_,
										defaultWindup_);
				pidController_.setSetpoint(0);
				pidController_.setMotor(1, 10, 11, 3, 6, 100, ioio_ );
				pidController_.setMotor(2, 12, 13, 4, 6, 100, ioio_ );
				pidController_.powerOn();
				led_.write(false);
				sleep(500);
				
				Log.d("Setup", "Got to the end of setup\n");
				enableUi(true);
			} catch (ConnectionLostException e) {
				enableUi(false);
				ioio_.disconnect();
				pidController_.close();
				throw e;
			} catch (InterruptedException e) {
				ioio_.disconnect();
				pidController_.close();
			}
		}
		
		public void loop() throws ConnectionLostException {
			try {
				//read button states/editText values
				//If toggle button is toggled pause PIDControl, otherwise unpause and update motor
				//If reset button has been pressed, reset internal PID stuff
				
				boolean motorUpdated = pidController_.updateMotors(0);
				//update the process variable text line
				setOutputText(Double.toString(pidController_.getProcessVar()));
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
//				seekBar_.setEnabled(enable);
//				toggleButton_.setEnabled(enable);
			}
		});
	}
	
	private void setOutputText(final String str1) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				PIDOutputView_.setText(str1);
			}
		});
	}
	
	private void setText(final String str1, final String str2) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
//				textView_.setText(str1);
//				loopRateView_.setText(str2);
			}
		});
	}
	
} 