package tweeter0830.boxbot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

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
import org.apache.commons.math3.*;
import org.apache.commons.math3.stat.StatUtils;
import org.apache.commons.math3.stat.correlation.Covariance;

/*Sensor Error Distributions*/
/* Accelerometer ~= .03
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
	private float[] OrientValues_ = new float[3];
	
	// Orientation Matrix
	private float[] OrientMatrix = new float[16];
	// Rotation Matrix
	private float[] RotatMatrix = new float[16];
	
	private FileWriter writer_;
	private SensorManager sensorManager_ = null;
	 
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
        orientXValue_ = (TextView) findViewById(R.id.orient_x_value);
        orientYValue_ = (TextView) findViewById(R.id.orient_y_value);
        orientZValue_ = (TextView) findViewById(R.id.orient_z_value);
      
        // Initialize accelerometer related view elements
        accelXValue_.setText("0.00");
        accelYValue_.setText("0.00");
        accelZValue_.setText("0.00");
      
        // Initialize magnetometer related view elements
        magnetXValue_.setText("0.00");
        magnetYValue_.setText("0.00");
        magnetZValue_.setText("0.00");
        
        // Initialize orientation related view elements
        orientXValue_.setText("0.00");
        orientYValue_.setText("0.00");
        orientZValue_.setText("0.00");

        //enableUi(false);
    }

    //Covariance 
    private final static int measNums_ = 1000;
    private int count_ = 0;
    private double[][] accelArray_ = new double[3][measNums_];
    private double[][] magneticArray_ = new double[3][measNums_];
    private boolean accelFlag_ = false;
    private boolean magneticFlag_ = false;
    // This method will update the UI on new sensor events
    public void onSensorChanged(SensorEvent sensorEvent) {
    	synchronized (this) {
    		if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
    			accelXValue_.setText(Float.toString(sensorEvent.values[0]));
    			accelYValue_.setText(Float.toString(sensorEvent.values[1]));
    			accelZValue_.setText(Float.toString(sensorEvent.values[2]));
    			AccelValues_ = sensorEvent.values.clone();
    			accelFlag_ = true;
    		}
    		if (sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
    			
    			magnetXValue_.setText(Float.toString(sensorEvent.values[0]));
    			magnetYValue_.setText(Float.toString(sensorEvent.values[1]));
    			magnetZValue_.setText(Float.toString(sensorEvent.values[2]));
    			MagnetValues_ = sensorEvent.values.clone();
    			magneticFlag_ = true;
    		}
    		if (AccelValues_ != null && MagnetValues_ != null) { 
                SensorManager.getRotationMatrix(RotatMatrix, OrientMatrix, AccelValues_, MagnetValues_); 
                SensorManager.getOrientation(RotatMatrix, OrientValues_); 
                OrientValues_[0] = OrientValues_[0]*180/3.14159f; 
                OrientValues_[1] = OrientValues_[1]*180/3.14159f; 
                OrientValues_[2] = OrientValues_[2]*180/3.14159f; 
                orientXValue_.setText(Float.toString(OrientValues_[0]));
                orientYValue_.setText(Float.toString(OrientValues_[1]));
                orientZValue_.setText(Float.toString(OrientValues_[2]));
    		}
    		if( accelFlag_ && magneticFlag_ && count_ < measNums_){
    			
    			accelArray_[0][count_] = AccelValues_[0];
    			accelArray_[1][count_] = AccelValues_[1];
    			accelArray_[2][count_] = AccelValues_[2];
    			magneticArray_[0][count_] = MagnetValues_[0];
    			magneticArray_[1][count_] = MagnetValues_[1];
    			magneticArray_[2][count_] = MagnetValues_[2];
    			accelFlag_ = magneticFlag_ = false;
    			if( writer_!=null)
	    			try {
						writer_.append(Float.toString(AccelValues_[0])).append(',')
							   .append(Float.toString(AccelValues_[1])).append(',')
							   .append(Float.toString(AccelValues_[2])).append(',')
							   .append(Float.toString(MagnetValues_[0])).append(',')
							   .append(Float.toString(MagnetValues_[1])).append(',')
							   .append(Float.toString(MagnetValues_[2])).append('\n');
						writer_.flush();
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
						Log.e(LOGTAG_, e.getMessage());
					}
    			count_++;
    			Log.v(LOGTAG_, "Count: " + Integer.toString(count_));
    		}
    		if( count_ == measNums_){
    			Covariance accelCovar = new Covariance();
    			Log.v(LOGTAG_, "Standard Deviation = "+accelCovar.covariance(accelArray_[0], accelArray_[0]));
    			Log.v(LOGTAG_, "Mean = "+StatUtils.mean(accelArray_[0]));
    			count_ = 0;
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
    	try {
    		File logFile = new File(Environment.getExternalStorageDirectory()+"/sensorLog.csv");
    		logFile.delete();
			writer_ = new FileWriter(logFile);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			Log.e(LOGTAG_, e.getMessage());
		}
    }
    
    @Override
    protected void onStop() {
    	super.onStop();
    } 

    @Override
    protected void onPause(){
    	// Unregister the listener
    	sensorManager_.unregisterListener(this);
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