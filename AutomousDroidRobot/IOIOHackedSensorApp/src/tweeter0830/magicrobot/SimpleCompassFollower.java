package tweeter0830.magicrobot;

//OLDTODO Figure out why the orientation being reported changes axis between screen turn on and off
	//I think that this was due to a failure in my app to unregister sensorListeners
//TODO find a concrete way to unregister my sensor listeners no matter what
//TODO add a button that when pressed will move the robot to an unpredictable (or set) theta
//TODO look up how to make a class multithread safe

//import ioio.lib.api.AnalogInput;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.DecimalFormat;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.Uart;
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
import tweeter0830.boxbot.ArduConnect;
import tweeter0830.pidcontrol.PID;
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
    
//	@Override
//	protected void onStop() {
//		if(IOIOThread_!=null)
//			IOIOThread_.close();
//		
//		Log.v(LOGTAG_, "Stopped\n");
//		super.onStop();
//	}
	
//	@Override
//	protected void onDestroy() {
//		if(IOIOThread_!=null)
//			IOIOThread_.close();
//		
//		Log.v(LOGTAG_, "Destroyed\n");
//		super.onDestroy();
//	}	
	
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
		private ArduConnect arduConnect_;
		private Uart arduUart_;
		private InputStream arduIn_;
		private OutputStream arduOut_;
		private PID leftMotorPID_;
		private PID rightMotorPID_;
		
		public void setup() throws ConnectionLostException {
			try {
				//SensorManager sm = (SensorManager) getSystemService(SENSOR_SERVICE);
				enableUi(true);
				Log.v("Setup", "Got to the beginning of setup\n");
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
				
				Log.v(LOGTAG_, "Establishing Connection with Arduino");
				arduConnect_.establishConnection();
				Log.v(LOGTAG_, "Connection Established");
				Log.v(LOGTAG_, "Got to the end of setup\n");
				enableUi(true);
			} catch (ConnectionLostException e) {
				enableUi(false);
				throw e;
//			} catch (InterruptedException e) {
//				ioio_.disconnect();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		long leftEncoder, rightEncoder, loopTime;
		double leftSpeed, rightSpeed;
		int leftPWM, rightPWM;
		
		public void loop() throws ConnectionLostException {
			try {
				//All of our input float vals should be updated through the UI process
				//If reset button has been pressed, reset internal PID stuff
				if(resetFlag_){
					leftMotorPID_.reset();
					rightMotorPID_.reset();
					//now that we've dealt with the flag, turn it back to false
					resetFlag_=false;
				}
				leftMotorPID_.setPID(	kpInput_.floatVal, 
										kiInput_.floatVal, 
										kdInput_.floatVal, 
										filterInput_.floatVal, 
										betaInput_.floatVal, 
										windupInput_.floatVal);
				rightMotorPID_.setPID(	kpInput_.floatVal, 
										kiInput_.floatVal, 
										kdInput_.floatVal, 
										filterInput_.floatVal, 
										betaInput_.floatVal, 
										windupInput_.floatVal);
				leftMotorPID_.setSetpoint(setpointInput_.floatVal);
				rightMotorPID_.setSetpoint(setpointInput_.floatVal);
				
				loopTime = System.nanoTime();
				//Log.v(LOGTAG_, "Started Loop @ "+loopTime);
				arduConnect_.updateEncoders();
				leftEncoder = arduConnect_.getLeftEncoder();
				rightEncoder = arduConnect_.getRightEncoder();
				arduConnect_.updateSpeed();
				leftSpeed = arduConnect_.getLeftSpeed();
				rightSpeed = arduConnect_.getRightSpeed();
				//Log.v(LOGTAG_, "Processing: "+proccesingButton_.isChecked());
				if(proccesingButton_.isChecked()){
					leftMotorPID_.updateProcessVar(leftSpeed, loopTime);
					rightMotorPID_.updateProcessVar(rightSpeed, loopTime);
					//Oh dear god, the output doesn't match what the PID is outputting. May cause errors
					leftPWM = (int)leftMotorPID_.outputUpdate();
					rightPWM = (int)rightMotorPID_.outputUpdate();
					arduConnect_.setPWM(leftPWM, 0);
				}
				else{
					arduConnect_.setPWM(0, 0);
				}
				//if( leftSpeed < 0){
					Log.v(LOGTAG_, "Left Speed: "+leftSpeed+"PWM: "+leftPWM);
				//}
				//Log.v(LOGTAG_, "Left Speed: "+leftSpeed);
				//Log.v(LOGTAG_, "Left PWM: "+leftPWM);
				setOutputText(leftPWM, leftEncoder, leftSpeed);
				//sleep(1);
//			} catch (InterruptedException e) {
//				ioio_.disconnect();
//			} catch (ConnectionLostException e) {
//				enableUi(false);
//				throw e;
			} catch (IOException e) {
				// TODO Auto-generated catch block
				Log.w(LOGTAG_, e.getMessage());
				arduConnect_.establishConnection();
			}
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