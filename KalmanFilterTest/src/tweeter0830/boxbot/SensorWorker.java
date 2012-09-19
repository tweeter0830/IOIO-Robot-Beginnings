package tweeter0830.boxbot;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class SensorWorker implements SensorEventListener {
	private SensorManager sensorManager_;
	private float[] rotatMatrix_ = new float[16];
	
	public float[] gyroValues_;
	public float[] accelValues_;
	public float[] orientValues_ = new float[3];
	
	private float[] gyroSum_;
	private float[] accelSum_;
	private float[] orientSum_;
	
	private int gyroCount_;
	private int accelCount_;
	private int orientCount_;
	
	public float[] aveGyroValues_;
	public float[] aveAccelValues_;
	public float[] aveOrientValues_;
	
	public boolean averaging_ = false;
	private long endTime_;
	
	public SensorWorker(SensorManager inSensorManager){
		// Get a reference to a SensorManager
		sensorManager_ = inSensorManager;
	}
    
	public void startAveraging(double seconds){
		//zero out average values
		aveGyroValues_ = new float[3];
		aveAccelValues_ = new float[3];
		aveOrientValues_ = new float[3];
		
		gyroSum_ = new float[3];
		accelSum_ = new float[3];
		orientSum_ = new float[3];
		
		gyroCount_ = 0;
		accelCount_ = 0;
		orientCount_ = 0;
		
		averaging_ = true;
		
		endTime_ = System.nanoTime() + (long)(1000000000*seconds);
	}
	
    public void registerListeners(int sensorRate){
    	sensorManager_.registerListener(this, sensorManager_.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION), sensorRate);
    	sensorManager_.registerListener(this, sensorManager_.getDefaultSensor(Sensor.TYPE_GYROSCOPE), sensorRate);
    	sensorManager_.registerListener(this, sensorManager_.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR), sensorRate);
    }
    
    public void unregListeners(){
    	sensorManager_.unregisterListener(this);
    }
    
    //I'm not using this method now
	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}

	@Override
	public void onSensorChanged(SensorEvent sensorEvent) {
	    long currentTime = System.nanoTime();
		if (sensorEvent.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
			accelValues_ = sensorEvent.values.clone();
			if(averaging_ && currentTime<endTime_){
				accelCount_++;
				accelSum_[0] += accelValues_[0];
				accelSum_[1] += accelValues_[1];
				accelSum_[2] += accelValues_[2];
			}
		}
		if (sensorEvent.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
			gyroValues_ = sensorEvent.values.clone();
			if(averaging_ && currentTime<endTime_){
				gyroCount_++;
				gyroSum_[0] += gyroValues_[0];
				gyroSum_[1] += gyroValues_[1];
				gyroSum_[2] += gyroValues_[2];
			}
		}
		if (sensorEvent.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
			SensorManager.getRotationMatrixFromVector(rotatMatrix_, sensorEvent.values);
			SensorManager.getOrientation(rotatMatrix_, orientValues_);
			if(averaging_ && currentTime<endTime_){
				orientCount_++;
				orientSum_[0] += orientValues_[0];
				orientSum_[1] += orientValues_[1];
				orientSum_[2] += orientValues_[2];
			}
		}
		if(averaging_ && currentTime>endTime_){
			aveAccelValues_[0] = accelSum_[0]/accelCount_;
			aveAccelValues_[1] = accelSum_[1]/accelCount_;
			aveAccelValues_[2] = accelSum_[2]/accelCount_;
			
			aveGyroValues_[0] = gyroSum_[0]/gyroCount_;
			aveGyroValues_[1] = gyroSum_[1]/gyroCount_;
			aveGyroValues_[2] = gyroSum_[2]/gyroCount_;
			
			aveOrientValues_[0] = orientSum_[0]/orientCount_;
			aveOrientValues_[1] = orientSum_[1]/orientCount_;
			aveOrientValues_[2] = orientSum_[2]/orientCount_;
			
			averaging_ = false;
		}
			
    }
}
