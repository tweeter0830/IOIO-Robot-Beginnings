package tweeter0830.motorLibrary;

public interface DcMotor {
	void moveForward(float speed);
	void moveBackward(float speed);
	void move(float speed);
	void brake();
	void standby();
	void powerOn();
}
