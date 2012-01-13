package tweeter0830.motorLibrary;

public interface DcMotor {
	void moveForward(double speed);
	void moveBackward(double speed);
	void move(double speed);
	void brake();
	void standby();
	void powerOn();
}
