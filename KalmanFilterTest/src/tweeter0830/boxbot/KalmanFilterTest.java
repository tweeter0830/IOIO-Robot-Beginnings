package tweeter0830.boxbot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;

import org.apache.commons.math3.analysis.function.Atan2;

import ioio.lib.android.AbstractIOIOActivity;
import ioio.lib.api.exception.ConnectionLostException;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
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
 * 		 = 0.15 @ 2.5 = 4.4%
 *       = 0.03 @ 0.7 = 4.2%
 */
public class KalmanFilterTest extends AbstractIOIOActivity implements SensorEventListener{
	
	public static final String LOGTAG_ = "Kalman Test";

	// Accelerometer X, Y, and Z values
	private TextView accelXValue_;
	private TextView accelYValue_;
	private TextView accelZValue_;
	private float[] AccelValues_;
	
	// Accelerometer X, Y, and Z values
	private TextView magnetXValue_;
	private TextView magnetYValue_;
	private TextView magnetZValue_;
	private float[] MagnetValues_;
	
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
	
	private float[] OrientValues_ = new float[3];
	
	// Orientation Matrix
	private float[] OrientMatrix = new float[16];
	// Rotation Matrix
	private float[] RotatMatrix = new float[16];
	// State Array
	private double[] currentState_ = new double[6];
	
	private FileWriter writer_;
	private SensorManager sensorManager_ = null;
	
	DiffDriveExtKF kalmanFilter_ = new DiffDriveExtKF(0.09, 0.21, 6371009, 34.6981);
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Get a reference to a SensorManager
        sensorManager_ = (SensorManager) getSystemService(SENSOR_SERVICE);
        setContentView(R.layout.main);
        
        // Capture accelerometer related view elements
        accelXValue_ = (TextView) findViewById(R.id.accel_x_value);
        accelYValue_ = (TextView) findViewById(R.id.accel_y_value);
        accelZValue_ = (TextView) findViewById(R.id.accel_z_value);

        // Capture magnetometer related view elements
        magnetXValue_ = (TextView) findViewById(R.id.magnet_x_value);
        magnetYValue_ = (TextView) findViewById(R.id.magnet_y_value);
        magnetZValue_ = (TextView) findViewById(R.id.magnet_z_value);
        
        // Capture orientation related view elements
//        orientXValue_ = (TextView) findViewById(R.id.orient_x_value);
//        orientYValue_ = (TextView) findViewById(R.id.orient_y_value);
//        orientZValue_ = (TextView) findViewById(R.id.orient_z_value);
      
        // Capture Kalman filter related view elements
        eastValue_ = (TextView) findViewById(R.id.orient_east_value);
        northValue_ = (TextView) findViewById(R.id.orient_north_value);
		velocityValue_ = (TextView) findViewById(R.id.orient_velo_value);
		accelerValue_ = (TextView) findViewById(R.id.orient_accel_value);
		thetaValue_ = (TextView) findViewById(R.id.orient_theta_value);
		thetaDotValue_ = (TextView) findViewById(R.id.orient_thetaDot_value);
        
        // Initialize accelerometer related view elements
        accelXValue_.setText("0.00");
        accelYValue_.setText("0.00");
        accelZValue_.setText("0.00");
      
        // Initialize magnetometer related view elements
        magnetXValue_.setText("0.00");
        magnetYValue_.setText("0.00");
        magnetZValue_.setText("0.00");
        
//        // Initialize orientation related view elements
//        orientXValue_.setText("0.00");
//        orientYValue_.setText("0.00");
//        orientZValue_.setText("0.00");
        // Initialize Kalman State related view elements
        eastValue_.setText("0.00");
        northValue_.setText("0.00");
        velocityValue_.setText("0.00");
		accelerValue_.setText("0.00");
		thetaValue_.setText("0.00");
		thetaDotValue_.setText("0.00");
        
        kalmanFilter_.initialize();
        double[] initialState = new double[6];
        kalmanFilter_.setState(initialState);
        double[] initalErrorCov = new double[6];
        initalErrorCov[0] = 0;
        initalErrorCov[1] = 0;
        initalErrorCov[2] = .001;
        initalErrorCov[3] = .001;
        initalErrorCov[4] = .1;
        initalErrorCov[5] = .001;
        kalmanFilter_.setSimpleP(initalErrorCov);
        double[] proccErrorCov = new double[6];
        proccErrorCov[0] = .2;
        proccErrorCov[1] = .2;
        proccErrorCov[2] = .01;
        proccErrorCov[3] = .3;
        proccErrorCov[4] = .2;
        proccErrorCov[5] = 0.1;
        kalmanFilter_.setSimpleQ(proccErrorCov);
        double[] measErrorCov = new double[6];
        measErrorCov[0] = 0;
        measErrorCov[1] = 0;
        measErrorCov[2] = 0.15;
        measErrorCov[3] = 0.15;
        measErrorCov[4] = 0.03;
        //This is a bad way to do this. The theta measurement doesn't have a normal distribution 
        measErrorCov[5] = 0.1;
        kalmanFilter_.setSimpleR(measErrorCov);
    }

    // This method will update the UI on new sensor events
    public void onSensorChanged(SensorEvent sensorEvent) {
    	final DecimalFormat fivePlaces = new DecimalFormat("0.00000");
    	synchronized (this) {
    		if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
    			AccelValues_ = sensorEvent.values.clone();
    			accelXValue_.setText(Float.toString(AccelValues_[0]));
    			accelYValue_.setText(Float.toString(AccelValues_[1]));
    			accelZValue_.setText(Float.toString(AccelValues_[2]));
    			kalmanFilter_.updateAccel(AccelValues_[1]);
    		}
    		if (sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
    			
    			MagnetValues_ = sensorEvent.values.clone();
    			magnetXValue_.setText(Float.toString(MagnetValues_[0]));
    			magnetYValue_.setText(Float.toString(MagnetValues_[1]));
    			magnetZValue_.setText(Float.toString(MagnetValues_[2]));
    			kalmanFilter_.updateHeading(Math.atan2(MagnetValues_[0], MagnetValues_[1]));
    		}
    		if (AccelValues_ != null && MagnetValues_ != null) { 
                SensorManager.getRotationMatrix(RotatMatrix, OrientMatrix, AccelValues_, MagnetValues_); 
                SensorManager.getOrientation(RotatMatrix, OrientValues_); 
                OrientValues_[0] = OrientValues_[0]*180/3.14159f; 
                OrientValues_[1] = OrientValues_[1]*180/3.14159f; 
                OrientValues_[2] = OrientValues_[2]*180/3.14159f; 
//                orientXValue_.setText(Float.toString(OrientValues_[0]));
//                orientYValue_.setText(Float.toString(OrientValues_[1]));
//                orientZValue_.setText(Float.toString(OrientValues_[2]));
                kalmanFilter_.getState(currentState_);
                eastValue_.setText(fivePlaces.format(currentState_[0]));
                northValue_.setText(fivePlaces.format(currentState_[1]));
                velocityValue_.setText(fivePlaces.format(currentState_[2]));
                accelerValue_.setText(fivePlaces.format(currentState_[3]));
                thetaValue_.setText(fivePlaces.format(currentState_[4]));
                thetaDotValue_.setText(fivePlaces.format(currentState_[5]));
    		}
    	}
    }

    // I've chosen to not implement this method
    public void onAccuracyChanged(Sensor arg0, int arg1) {
    	// TODO Auto-generated method stub
    }

    @Override
    protected void onResume() {
    	super.onResume();
    	// Register this class as a listener for the accelerometer sensor
    	sensorManager_.registerListener(this, sensorManager_.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_FASTEST);
    	// ...and the magnetic sensor
    	sensorManager_.registerListener(this, sensorManager_.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_FASTEST);
    	//try to open a log file
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

    @Override
    protected void onPause(){
    	// Unregister the listener
    	sensorManager_.unregisterListener(this);
    	//If we successfully opened a file, kill it
    	if(writer_!=null)
	    	try {
				writer_.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    	super.onPause();
    }
	class IOIOThread extends AbstractIOIOActivity.IOIOThread {
		@Override
		public void setup() throws ConnectionLostException {
//			try {
//			} catch (ConnectionLostException e) {
//				enableUi(false);
//				throw e;
//			}
		}
		
		@Override
		public void loop() throws ConnectionLostException {
//			try {
//			} catch (InterruptedException e) {
//				ioio_.disconnect();
//			} catch (ConnectionLostException e) {
//				enableUi(false);
//				throw e;
//			} catch (IOException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
		}
	}

	@Override
	protected AbstractIOIOActivity.IOIOThread createIOIOThread() {
		return new IOIOThread();
	}

//	private void enableUi(final boolean enable) {
//		runOnUiThread(new Runnable() {
//			@Override
//			public void run() {
//				seekBar_.setEnabled(enable);
//				toggleButton_.setEnabled(enable);
//			}
//		});
//	}
//	
//	private void setText(final String str) {
//		runOnUiThread(new Runnable() {
//			@Override
//			public void run() {
//				textView_.setText(str);
//			}
//		});
//	}
}