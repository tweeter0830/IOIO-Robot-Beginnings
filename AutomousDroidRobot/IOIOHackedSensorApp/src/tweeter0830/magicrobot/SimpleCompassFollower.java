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
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.KeyEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.ToggleButton;
import android.view.View.OnKeyListener;
import android.util.Log;
//Libraries for creating a motor driver class
import tweeter0830.pidcontrol.PIDController;
import tweeter0830.pidcontrol.Encoder;

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
	private TextView countOutputView_;
	private TextView speedOutputView_;
	
	private volatile boolean resetFlag_=false;
	private volatile floatWrapper setpointInput_ = new floatWrapper();
	private volatile floatWrapper kpInput_ = new floatWrapper();
	private volatile floatWrapper kiInput_ = new floatWrapper();
	private volatile floatWrapper kdInput_ = new floatWrapper();
	private volatile floatWrapper betaInput_ = new floatWrapper();
	private volatile floatWrapper filterInput_ = new floatWrapper();
	private volatile floatWrapper windupInput_ = new floatWrapper();
	private volatile boolean procButtonVal_;
	
	private IOIOMotoThread IOIOThread_ = null;
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.v(LOGTAG_, "Created\n");
        
        setContentView(R.layout.main);
        
        //find all of our views
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
        countOutputView_ = 	(TextView)findViewById(R.id.textViewCount);
        speedOutputView_ = 	(TextView)findViewById(R.id.textViewSpeed);
        
        //Get default values for our inputs
        kpInput_.floatVal = Float.valueOf(this.getString(R.string.defaultKpVal).trim()).floatValue();
        kiInput_.floatVal = Float.valueOf(this.getString(R.string.defaultKiVal).trim()).floatValue();
        kdInput_.floatVal = Float.valueOf(this.getString(R.string.defaultKdVal).trim()).floatValue();
        betaInput_.floatVal = Float.valueOf(this.getString(R.string.defaultBetaVal).trim()).floatValue();
        filterInput_.floatVal = Float.valueOf(this.getString(R.string.defaultFilterVal).trim()).floatValue();
        windupInput_.floatVal = Float.valueOf(this.getString(R.string.defaultWindupVal).trim()).floatValue();
        setpointInput_.floatVal = Float.valueOf(this.getString(R.string.defaultSetpointVal).trim()).floatValue();
    	
        //A listener to handle when the user presses the reset button
    	resetButton_.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
            	resetFlag_ = true;
            }
        });
    	
    	//Assign listeners to update the appropriate float values when their respective views change
    	kpView_.setOnKeyListener(new assignEditTextListener(kpView_,kpInput_));
    	kiView_.setOnKeyListener(new assignEditTextListener(kiView_,kiInput_));
    	kdView_.setOnKeyListener(new assignEditTextListener(kdView_,kdInput_));
    	betaView_.setOnKeyListener(new assignEditTextListener(betaView_,betaInput_));
    	filterView_.setOnKeyListener(new assignEditTextListener(filterView_,filterInput_));
    	windupView_.setOnKeyListener(new assignEditTextListener(windupView_,windupInput_));
    	setpointView_.setOnKeyListener(new assignEditTextListener(setpointView_,setpointInput_));
    	
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
	
	class assignEditTextListener implements OnKeyListener{

		private EditText inputView_;
		private floatWrapper inFloat_;
		
		public assignEditTextListener(EditText inputView, floatWrapper inFloat ){
			inputView_ = inputView;
			inFloat_ = inFloat;
		}
		@Override
		public boolean onKey(View v, int keyCode, KeyEvent event) {
	        // If the event is a key-down event on the "enter" button
	        if ((event.getAction() == KeyEvent.ACTION_DOWN) &&
	            (keyCode == KeyEvent.KEYCODE_ENTER)) {
	        	// Perform action on key press
		        try{
		        	inFloat_.floatVal = Float.parseFloat(inputView_.getText().toString());
		        	return false;
		        }catch (NumberFormatException nfe){
		        	   // bad data
		        	inputView_.setText(Float.toString(inFloat_.floatVal));
	        		return false;
		        }
	        }
	        if((event.getAction() == KeyEvent.ACTION_UP) &&
	            (keyCode == KeyEvent.KEYCODE_ENTER) ){
	        	View nextView = inputView_.focusSearch(View.FOCUS_RIGHT);
	        	if(nextView!= null)
	        		inputView_.focusSearch(View.FOCUS_RIGHT).requestFocus();
	        	
	        	return false;
	        }
	        return ((keyCode < KeyEvent.KEYCODE_0)
	        	|| (keyCode > KeyEvent.KEYCODE_9))
	        	&& ((keyCode != KeyEvent.KEYCODE_PERIOD)
	        	|| (keyCode != KeyEvent.KEYCODE_DEL)
	        	|| (keyCode != KeyEvent.KEYCODE_BACK));
	    }
	}
	
	private class floatWrapper{
		public float floatVal;
	}
	
	class IOIOMotoThread extends AbstractIOIOActivity.IOIOThread {
		
		private DigitalOutput led_;
		private PIDController pidController_;
		private Encoder leftEncoder_;
		public void setup() throws ConnectionLostException {
			try {
				SensorManager sm = (SensorManager) getSystemService(SENSOR_SERVICE);
				
				led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
				
				pidController_ = new PIDController(sm);
				pidController_.setPID(	kpInput_.floatVal,
										kiInput_.floatVal, 
										kdInput_.floatVal, 
										filterInput_.floatVal,
										betaInput_.floatVal,
										windupInput_.floatVal);
				pidController_.setSetpoint(setpointInput_.floatVal);
				pidController_.setMotor(1, 42, 43, 46, 38, 100, ioio_ );
				pidController_.setMotor(2, 41, 40, 45, 38, 100, ioio_ );
				pidController_.powerOn();
				led_.write(false);
				Encoder leftEncoder_ = new Encoder(ioio_, 31, 100);
				
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
				//All of our input float vals should be updated through the UI process
				//If reset button has been pressed, reset internal PID stuff
				if(resetFlag_){
					pidController_.internalPID_.reset();
					//now that we've dealt with the flag, turn it back to false
					resetFlag_=false;
				}
				pidController_.internalPID_.setPID(	kpInput_.floatVal, 
													kiInput_.floatVal, 
													kdInput_.floatVal, 
													filterInput_.floatVal, 
													betaInput_.floatVal, 
													windupInput_.floatVal);
				Log.d(LOGTAG_,"kpInput = " + kpInput_.toString() + "\tGiuValue: " + kpView_.getText().toString());
				pidController_.internalPID_.setSetpoint(setpointInput_.floatVal);
				//If toggle button is toggled pause PIDControl, otherwise unpause and update motor
				pidController_.pause(!proccesingButton_.isChecked());
				led_.write(!proccesingButton_.isChecked());
				//PIDController won't actually update the motor if paused
				boolean motorUpdated = pidController_.updateMotors(0);
				//update the process variable text line
				setOutputText(pidController_.getProcessVar(), leftEncoder_.getRotations(), leftEncoder_.getSpeed() );
				
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
	private void setOutputText(final double outputVar, final double count, final double speed) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				DecimalFormat twoPlaces = new DecimalFormat("0.00");
				PIDOutputView_.setText(twoPlaces.format(outputVar));
				countOutputView_.setText(twoPlaces.format(count));
				speedOutputView_.setText(twoPlaces.format(speed));
			}
		});
	}
} 