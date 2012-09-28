package tweeter0830.boxbot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.DecimalFormat;
import java.util.Arrays;

import tweeter0830.pidcontrol.PID;
import tweeter0830.boxbot.ArduConnect;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;
import ioio.lib.api.Uart;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.widget.TextView;

/******Chassis Physical Parameters******/
/*
 * Wheel Diameter ~= 9 cm
 * Wheel Base ~= 21 cm
 */
/******Other Parameters******/
/*
 * Rough Longitude = 34.6981° N
 * Earth Radius = 6,371.009 kilometers
 */
/******Sensor Error Distributions******/
/* 
 * Accelerometer ~= .03
 * Magnetometer ~= .46
 * Wheel = 0.31 @ 5.4 = 5.6%
 *       = 0.15 @ 2.5 = 4.4%
 *       = 0.03 @ 0.7 = 4.2%
 */

public class KalmanFilterTest extends IOIOActivity{
	public static final String LOGTAG_ = "Kalman Test";
	public static final boolean FILELOGGING_ = false;
	
	private double wheelDiam_ = 0.09; //meters
	private double wheelBase_ = 0.21; //meters
	
	// Accelerometer X, Y, and Z values
	private TextView accelXValue_;
	private TextView accelYValue_;
	private TextView accelZValue_;
	
	// Accelerometer X, Y, and Z values
	private TextView gyroXValue_;
	private TextView gyroYValue_;
	private TextView gyroZValue_;
	
	// Orientation X, Y, and Z values
	private TextView orientXValue_;
	private TextView orientYValue_;
	private TextView orientZValue_;
	
	// Kalman Filter Values
	private TextView eastValue_;
	private TextView northValue_;
	private TextView velocityValue_;
	private TextView accelerValue_;
	private TextView thetaValue_;
	private TextView thetaDotValue_;
	
	// State Array
	private double[] currentState_ = new double[6];
	private double[] currentMeas_ = new double[7];
	private double[] measZeroOffset_ = new double[7];
	
	private FileWriter writer_;
	
	//A SensorWorker to take care of getting the sensor readings from the phone
	private SensorWorker sensorWorker_;
	//Create a differential drive Kalman filter based on the dimensions of this robot 
	private DiffDriveExtKF kalmanFilter_ = new DiffDriveExtKF(0.09, 0.21, 6371009, 34.6981);
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        //Create a SensorWorker to take care of getting the sensor readings from the phone
    	sensorWorker_ = new SensorWorker( (SensorManager)getSystemService(SENSOR_SERVICE) );
    	
        // Capture accelerometer related view elements
        accelXValue_ = (TextView) findViewById(R.id.accel_x_value);
        accelYValue_ = (TextView) findViewById(R.id.accel_y_value);
        accelZValue_ = (TextView) findViewById(R.id.accel_z_value);

        // Capture gyroscope related view elements
        gyroXValue_ = (TextView) findViewById(R.id.gyroscope_x_value);
        gyroYValue_ = (TextView) findViewById(R.id.gyroscope_y_value);
        gyroZValue_ = (TextView) findViewById(R.id.gyroscope_z_value);
        
        // Capture orientation related view elements
        orientXValue_ = (TextView) findViewById(R.id.orientation_x_value);
    	orientYValue_ = (TextView) findViewById(R.id.orientation_y_value);
        orientZValue_ = (TextView) findViewById(R.id.orientation_z_value);
      
        // Capture Kalman filter related view elements
        eastValue_ = (TextView) findViewById(R.id.state_east_value);
        northValue_ = (TextView) findViewById(R.id.state_north_value);
		velocityValue_ = (TextView) findViewById(R.id.state_velo_value);
		accelerValue_ = (TextView) findViewById(R.id.state_accel_value);
		thetaValue_ = (TextView) findViewById(R.id.state_theta_value);
		thetaDotValue_ = (TextView) findViewById(R.id.state_thetaDot_value);
        
        // Initialize accelerometer related view elements
        accelXValue_.setText("0.00");
        accelYValue_.setText("0.00");
        accelZValue_.setText("0.00");
      
        // Initialize gyroscope related view elements
        gyroXValue_.setText("0.00");
        gyroYValue_.setText("0.00");
        gyroZValue_.setText("0.00");
        
        // Initialize orientation related view elements
        orientXValue_.setText("0.00");
        orientYValue_.setText("0.00");
        orientZValue_.setText("0.00");
        
        // Initialize Kalman State related view elements
        eastValue_.setText("0.00");
        northValue_.setText("0.00");
        velocityValue_.setText("0.00");
		accelerValue_.setText("0.00");
		thetaValue_.setText("0.00");
		thetaDotValue_.setText("0.00");
        
        kalmanFilter_.initialize();
        
        //set the Kalman's initial state to zero
        double[] initialState = new double[6];
        kalmanFilter_.setState(initialState);
        
        //Set the initial error covariance matrix
        double[] initalErrorCov = new double[6];
        initalErrorCov[0] = 0;
        initalErrorCov[1] = 0;
        initalErrorCov[2] = .001;
        initalErrorCov[3] = .001;
        initalErrorCov[4] = .1;
        initalErrorCov[5] = .001;
        kalmanFilter_.setDiagonal(kalmanFilter_.P_, initalErrorCov);
        
        //Set the process error covariance matrix
        double[] proccErrorCov = new double[6];
        proccErrorCov[0] = .2;
        proccErrorCov[1] = .2;
        proccErrorCov[2] = .01;
        proccErrorCov[3] = .3;
        proccErrorCov[4] = .2;
        proccErrorCov[5] = 0.1;
        kalmanFilter_.setDiagonal(kalmanFilter_.Q_, proccErrorCov);
        
        //Set the measurement error covariance matrix
        double[] measErrorCov = new double[7];
        measErrorCov[0] = 999999999; //GPS X
        measErrorCov[1] = 999999999; //GPS Y
        measErrorCov[2] = 0.15*wheelDiam_; //Wheel velocity
        measErrorCov[3] = 999999999;//0.00584;		 //Acceleration //Average = 0.01 //STDEV = 0.00584
        measErrorCov[4] = 0.0489;		 //Orientation //Average = - //STDEV = 0.0489
        measErrorCov[5] = 0.00339;		 //Gyro Theta Dot ////Average = -0.056 //STDEV = 0.00339
        measErrorCov[6] = 0.15*wheelDiam_/wheelBase_; //Wheel Theta Dot
        kalmanFilter_.setDiagonal(kalmanFilter_.R_,measErrorCov);
        
        measZeroOffset_[0] = 0;
        measZeroOffset_[1] = 0;
        measZeroOffset_[2] = 0;
        measZeroOffset_[3] = 0.01;
        measZeroOffset_[4] = 0;
        measZeroOffset_[5] = -0.056;
        measZeroOffset_[6] = 0;
    }

	    
    @Override
    protected void onResume() {
    	super.onResume();
    	//TODO Speed this up when I think the program can handle it
    	sensorWorker_.registerListeners(SensorManager.SENSOR_DELAY_GAME);
    	//try to open a log file
    	if(FILELOGGING_){
	    	try {
	    		File logFile = new File(Environment.getExternalStorageDirectory()+"/sensorLog.csv");
	    		//clear the file if it exists
	    		logFile.delete();
				writer_ = new FileWriter(logFile);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				Log.e(LOGTAG_, e.getMessage());
			}
    	}
    }
    
    @Override
    protected void onPause(){
    	// Unregister the listener
    	sensorWorker_.unregListeners();
    	
    	//If we successfully opened a file, kill it
    	if(writer_!=null)
	    	try {
				writer_.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				Log.e(LOGTAG_, e.getMessage());
			}
    	super.onPause();
    }
    
	class RobotLoop extends BaseIOIOLooper {
		private DigitalOutput led_;
		
		private ArduConnect arduConnect_;
		private Uart arduUart_;
		private InputStream arduIn_;
		private OutputStream arduOut_;
		private PID leftMotorPID_;
		private PID rightMotorPID_;
		/**
		 * Called every time a connection with IOIO has been established.
		 * 
		 * @throws ConnectionLostException
		 *             When IOIO connection is lost.
		 * 
		 * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#setup()
		 */
		@Override
		protected void setup() throws ConnectionLostException {
			led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
			arduUart_ = ioio_.openUart(6, 7, 115200, Uart.Parity.NONE, Uart.StopBits.ONE);
			arduIn_ = arduUart_.getInputStream();
			arduOut_ = arduUart_.getOutputStream();
			arduConnect_ = new ArduConnect(arduIn_, arduOut_);
			Log.v(LOGTAG_,"Establishing Connection");
			arduConnect_.establishConnection();
			Log.v(LOGTAG_,"Connection Established");
			sensorWorker_.startAveraging(10.0);
			Log.v(LOGTAG_, "Averaging");
			while(sensorWorker_.averaging_){
			}
			Log.v(LOGTAG_, "AverageAccel = "+ sensorWorker_.aveAccelValues_[1]);
			Log.v(LOGTAG_, "AverageGyro = "+ sensorWorker_.aveGyroValues_[2]);
			measZeroOffset_[3] = sensorWorker_.aveAccelValues_[1];
			measZeroOffset_[5] = sensorWorker_.aveGyroValues_[2];
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
		public void loop() throws ConnectionLostException {
			//Update current sensor readings from the arduino
			try {
				arduConnect_.updateSpeed();
			} catch (IOException e1) {
				ioio_.disconnect();
			}
			//TODO currentMeas_[0] = GPS X
	    	//TODO currentMeas_[1] = GPS Y
	    	currentMeas_[2] = (arduConnect_.getLeftSpeed()+arduConnect_.getRightSpeed())/2*wheelDiam_;
	    	currentMeas_[3] = sensorWorker_.accelValues_[1]-measZeroOffset_[3];
	    	currentMeas_[4] = sensorWorker_.orientValues_[0];
	    	currentMeas_[5] = sensorWorker_.gyroValues_[2]-measZeroOffset_[5];
	    	currentMeas_[6] = (arduConnect_.getRightSpeed()-arduConnect_.getLeftSpeed())/wheelBase_*wheelDiam_;
	    	updateSensorText(sensorWorker_);
	    	
	    	kalmanFilter_.updateAll(currentMeas_);
	        kalmanFilter_.getState(currentState_);
	        updateStateText(currentState_);
	        if(FILELOGGING_){
		        try{
		        	writer_.write(Arrays.toString(currentMeas_)+'\n');
				} catch (IOException e) {
					e.printStackTrace();
					Log.e(LOGTAG_, e.getMessage());
				}
	        }
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
		}
	}
    
	 /**
	 * A method to create our IOIO thread.
	 * 
	 * @see ioio.lib.util.AbstractIOIOActivity#createIOIOThread()
	 */
	@Override
	protected IOIOLooper createIOIOLooper() {
		return new RobotLoop();
	}
    
    private void updateSensorText(final SensorWorker sensorWorker) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
		        final DecimalFormat fivePlaces = new DecimalFormat("0.00000");
		        
		        accelXValue_.setText(fivePlaces.format(sensorWorker.accelValues_[0]));
		        accelYValue_.setText(fivePlaces.format(sensorWorker.accelValues_[1]));
		        accelZValue_.setText(fivePlaces.format(sensorWorker.accelValues_[2]));
		        
		        gyroXValue_.setText(fivePlaces.format(sensorWorker.gyroValues_[0]));
		        gyroYValue_.setText(fivePlaces.format(sensorWorker.gyroValues_[1]));
		        gyroZValue_.setText(fivePlaces.format(sensorWorker.gyroValues_[2]));
		        
		        orientXValue_.setText(fivePlaces.format(sensorWorker.orientValues_[0]));
		        orientYValue_.setText(fivePlaces.format(sensorWorker.orientValues_[1]));
		        orientZValue_.setText(fivePlaces.format(sensorWorker.orientValues_[2]));
			}
		});
	}
    
    private void updateStateText(final double[] states) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
		        final DecimalFormat fivePlaces = new DecimalFormat("0.00000");
                eastValue_.setText(fivePlaces.format(states[0]));
                northValue_.setText(fivePlaces.format(states[1]));
                velocityValue_.setText(fivePlaces.format(states[2]));
                accelerValue_.setText(fivePlaces.format(states[3]));
                thetaValue_.setText(fivePlaces.format(states[4]/3.14159*180.0));
                thetaDotValue_.setText(fivePlaces.format(states[5]/3.14159*180.0));
			}
		});
	}
}