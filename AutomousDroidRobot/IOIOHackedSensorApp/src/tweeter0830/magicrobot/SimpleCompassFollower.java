package tweeter0830.magicrobot;

//OLDTODO Figure out why the orientation being reported changes axis between screen turn on and off
	//I think that this was due to a failure in my app to unregister sensorListeners
//TODO find a concrete way to unregister my sensor listeners no matter what
//TODO add a button that when pressed will move the robot to an unpredictable (or set) theta
//TODO look up how to make a class multithread safe

//import ioio.lib.api.AnalogInput;
import java.text.DecimalFormat;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
//import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.android.AbstractIOIOActivity;
import ioio.lib.android.ActivityDependentIOIOConnectionBootstrap;
import ioio.lib.spi.IOIOConnectionBootstrap;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
import android.util.Log;
//Libraries for creating a motor driver class
import tweeter0830.pidcontrol.PIDController;

public class SimpleCompassFollower extends AbstractIOIOActivity {
	public static final String LOGTAG_ = "Compass Follow";
	
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
			
	public volatile boolean resetFlag_=false; 
	
	private IOIOMotoThread IOIOThread_ = null;
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.v(LOGTAG_, "Created\n");
        
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
    	
    	resetButton_.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
            	resetFlag_ = true;
            }
        });
    	
        enableUi(false);
    }
    
	@Override
	protected void onStop() {
		if(IOIOThread_!=null)
			IOIOThread_.close();
		
		Log.v(LOGTAG_, "Stopped\n");
		super.onStop();
	}
	
	@Override
	protected void onDestroy() {
		if(IOIOThread_!=null)
			IOIOThread_.close();
		
		Log.v(LOGTAG_, "Destroyed\n");
		super.onDestroy();
	}	
	
	class IOIOMotoThread extends AbstractIOIOActivity.IOIOThread {
		private DigitalOutput led_;
		
		private float setpointInput_;
		private float kpInput_;
		private float kiInput_;
		private float kdInput_;
		private float betaInput_;
		private float filterInput_;
		private float windupInput_;
		private boolean procButtonVal_;
		
		private double loopRate_;
		private PIDController pidController_;
		
		public void setup() throws ConnectionLostException {
			try {
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
				//If reset button has been pressed, reset internal PID stuff
				if(resetFlag_){
					pidController_.internalPID_.reset();
					//now that we've dealt with the flag, turn it back to false
					resetFlag_=false;
				}
				//If toggle button is toggled pause PIDControl, otherwise unpause and update motor
				pidController_.pause(!proccesingButton_.isChecked());
				if(proccesingButton_.isChecked())
				{
					//read toggle button state/editText values
					readInputs();
					//set PidVals
					pidController_.internalPID_.setPID(	kpInput_, 
														kiInput_, 
														kdInput_, 
														filterInput_, 
														betaInput_, 
														windupInput_);
					pidController_.internalPID_.setSetpoint(setpointInput_);
				}
				//PIDController won't actually update the motor if paused
				boolean motorUpdated = pidController_.updateMotors(0);
				//update the process variable text line
				setOutputText(pidController_.getProcessVar());
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
		
		private void readInputs(){
			setpointInput_ = Double.valueOf(setpointView_.getText().toString().trim()).floatValue();
			kpInput_ = Double.valueOf(kpView_.getText().toString().trim()).floatValue();
			kiInput_ = Double.valueOf(kiView_.getText().toString().trim()).floatValue();
			kdInput_ = Double.valueOf(kdView_.getText().toString().trim()).floatValue();
			betaInput_ = Double.valueOf(betaView_.getText().toString().trim()).floatValue();
			filterInput_ = Double.valueOf(filterView_.getText().toString().trim()).floatValue();
			windupInput_ = Double.valueOf(windupView_.getText().toString().trim()).floatValue();
		}
		
		public void close(){
			if(pidController_!=null)
				pidController_.close();
		}
	}

	@Override
	protected AbstractIOIOActivity.IOIOThread createIOIOThread() {
		IOIOThread_ = new IOIOMotoThread();
		return IOIOThread_;
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
	
	private void setOutputText(final double outputVar) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				DecimalFormat twoPlaces = new DecimalFormat("0.00");
				PIDOutputView_.setText(twoPlaces.format(outputVar));
			}
		});
	}
} 