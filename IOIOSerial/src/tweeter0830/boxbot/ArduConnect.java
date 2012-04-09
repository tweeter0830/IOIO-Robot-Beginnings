package tweeter0830.boxbot;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import android.util.Log;

public class ArduConnect {
	private static final boolean ISLOGGING = true;
	private static final String LOGTAG_ = "Compass Follow";
	
	private InputStream arduIn_;
	private OutputStream arduOut_;
	
	private Mode mode_;
	private long leftEncoder_ = 0;
	private long rightEncoder_ = 0;
	private double leftSpeed_ = 0;
	private double rightSpeed_ = 0;
	
	public enum Mode{
		SPEED,PWM
	}
	public ArduConnect(InputStream streamIn, OutputStream streamOut){
		arduIn_ = streamIn;
		arduOut_ = streamOut;
	}
	public void setMode(Mode modeIn) throws IOException{
		arduOut_.write('0');
		mode_ = modeIn;
		if( mode_ == Mode.SPEED ){
			arduOut_.write('1');
		}else{
			arduOut_.write('0');
		}
		arduOut_.flush();
		if(ISLOGGING){
			Log.v(LOGTAG_,"Mode Set");
		}
	}
	public void updateEncoders() throws IOException{
		byte bufferIn[]= new byte[8];
		arduOut_.write('1');
		int bufLength = arduIn_.read(bufferIn, 0, 8);
		leftEncoder_ = bytesToLong(bufferIn,0);
		rightEncoder_= bytesToLong(bufferIn,4);
		if(ISLOGGING){
			Log.v(LOGTAG_,"Enc Buffer: "+bufferIn[0]+' '+bufferIn[1]+' '+bufferIn[2]+' '+
									     bufferIn[3]+' '+bufferIn[4]+' '+bufferIn[5]+' '+bufferIn[6]+' '+bufferIn[7]);
			Log.v(LOGTAG_,"Length: "+bufLength);
			Log.v(LOGTAG_,"Left Encoder: "+leftEncoder_);
			Log.v(LOGTAG_,"Right Encoder: "+rightEncoder_);
		}
	}
	public void updateSpeed() throws IOException{
		byte bufferIn[]= new byte[8];
		arduOut_.write('2');
		int bufLength = arduIn_.read(bufferIn, 0, 8);
		leftSpeed_ = ((double)bytesToLong(bufferIn,0))/10000;
		rightSpeed_= ((double)bytesToLong(bufferIn,4))/10000;
		if(ISLOGGING){
			Log.v(LOGTAG_,"Spd Buffer: "+bufferIn[0]+' '+bufferIn[1]+' '+bufferIn[2]+' '+
									     bufferIn[3]+' '+bufferIn[4]+' '+bufferIn[5]+' '+bufferIn[6]+' '+bufferIn[7]);
			Log.v(LOGTAG_,"Length: "+bufLength);
			Log.v(LOGTAG_,"Left Speed: "+leftSpeed_);
			Log.v(LOGTAG_,"Right Speed: "+rightSpeed_);
		}
	}
	public void setPWM(int leftPWM,int rightPWM) throws IOException{
		byte bufferOut[] = new byte[4];
		intToBytes(leftPWM,bufferOut,0);
		intToBytes(rightPWM,bufferOut,2);
		if(ISLOGGING){
			Log.v(LOGTAG_,"Int 1 Buffer: "+((int)bufferOut[0] & 0xff)+' '+((int)bufferOut[1] & 0xff));
			Log.v(LOGTAG_,"Int 2 Buffer: "+((int)bufferOut[2] & 0xff)+' '+((int)bufferOut[3] & 0xff));
			Log.v(LOGTAG_,"Int 1 In: "+leftPWM);
			Log.v(LOGTAG_,"Int 2 In: "+rightPWM);
		}
		arduOut_.write('3');
		arduOut_.write(bufferOut);
	}
	public void setSpeed(double leftSpeed,double rightSpeed) throws IOException{
		byte bufferOut[] = new byte[8];
		longToBytes((long)(leftSpeed*10000),bufferOut,0);
		longToBytes((long)(rightSpeed*10000),bufferOut,4);
		if(ISLOGGING){
			Log.v(LOGTAG_,"Speed 1 Out: "+leftSpeed);
			Log.v(LOGTAG_,"Speed 2 Out: "+rightSpeed);
			Log.v(LOGTAG_,"Speed 1 Buffer: "+((long)bufferOut[0] & 0xff)+' '+((long)bufferOut[1] & 0xff)+' '+
										     ((long)bufferOut[2] & 0xff)+' '+((long)bufferOut[3] & 0xff));
		}
		arduOut_.write('4');
		arduOut_.write(bufferOut);
	}
	public void setGains(float kp,float ki, float kd) throws IOException{
		byte bufferOut[] = new byte[6];
		intToBytes((int)(kp*100),bufferOut,0);
		intToBytes((int)(ki*100),bufferOut,2);
		intToBytes((int)(kd*100),bufferOut,4);
		if(ISLOGGING){
			Log.v(LOGTAG_,"kp: "+kp);
			Log.v(LOGTAG_,"ki: "+ki);
			Log.v(LOGTAG_,"kd: "+kd);
		}
		arduOut_.write('5');
		arduOut_.write(bufferOut);
	}
	private void intToBytes(int intIn, byte arrayIn[], int startIndex){
		arrayIn[startIndex] = (byte)(intIn>>0);
		arrayIn[startIndex+1] = (byte)(intIn>>8);
	}
	void longToBytes(long longIn, byte arrayIn[], int startIndex){
		arrayIn[startIndex] = (byte)(longIn>>0);
		arrayIn[startIndex+1] = (byte)(longIn>>8);
		arrayIn[startIndex+2] = (byte)(longIn>>16);
		arrayIn[startIndex+3] = (byte)(longIn>>24);

	}
	private long bytesToLong(byte arrayIn[],int startIndex){
		long longOut = 0;
		longOut = (int)arrayIn[startIndex] & 0xff;
		longOut = (((int)arrayIn[startIndex+1] & 0xff)<<8)+longOut;
		longOut = (((int)arrayIn[startIndex+2] & 0xff)<<16)+longOut;
		longOut = (((int)arrayIn[startIndex+3] & 0xff)<<24)+longOut;
	        return longOut;
	}
}
