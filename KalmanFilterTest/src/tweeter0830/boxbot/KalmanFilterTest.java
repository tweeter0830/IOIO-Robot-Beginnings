package tweeter0830.boxbot;

import ioio.lib.android.AbstractIOIOActivity;
import ioio.lib.api.exception.ConnectionLostException;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.TextView;

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

    // This method will update the UI on new sensor events
    public void onSensorChanged(SensorEvent sensorEvent) {
    	synchronized (this) {
    		if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
    			accelXValue_.setText(Float.toString(sensorEvent.values[0]));
    			accelYValue_.setText(Float.toString(sensorEvent.values[1]));
    			accelZValue_.setText(Float.toString(sensorEvent.values[2]));
    			AccelValues_ = sensorEvent.values.clone();
    		}
    		if (sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
    			magnetXValue_.setText(Float.toString(sensorEvent.values[0]));
    			magnetYValue_.setText(Float.toString(sensorEvent.values[1]));
    			magnetZValue_.setText(Float.toString(sensorEvent.values[2]));
    			MagnetValues_ = sensorEvent.values.clone();
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
    }
    
    @Override
    protected void onStop() {
    	// Unregister the listener
    	sensorManager_.unregisterListener(this);
    	super.onStop();
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