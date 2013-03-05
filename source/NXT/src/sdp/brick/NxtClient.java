package sdp.brick;

import lejos.nxt.*;
import lejos.nxt.comm.*;
import java.io.*;


public class NxtClient {
	private static final boolean useBluetooth = true;
	private static final byte    OP_SPEEDS    = 0x0;
	private static final byte    OP_KICK      = 0x1;
	private static final byte    OP_SHUTDOWN  = 0x2;
	
	enum KickingPhase {
		NOT_KICKING,
		UP,
		BACK
	}
	
	private NXTRegulatedMotor   kickerMotor = Motor.C;
	private NXTRegulatedMotor[] wheelMotors = { Motor.A, Motor.B };
	private float[]             wheelSpeeds = { 0.0f, 0.0f };
	private TouchSensor         touchSensors[] = { new TouchSensor( SensorPort.S1 ), new TouchSensor( SensorPort.S4 ) };
	
	private NXTConnection       connection;
	private DataInputStream     inputStream;
	private boolean             running, connected;
	private KickingPhase        kickingPhase;
	private long                kickingEndsAt;
	
	private long                backOffEndsAt;
	
	
	private boolean isBackingOff() {
		return System.currentTimeMillis() < backOffEndsAt;
	}
	
	private float   getWheelMaxSpeed() {
		return Math.min( wheelMotors[0].getMaxSpeed(), wheelMotors[1].getMaxSpeed() );
	}
	
	public void run() {
		this.running   = true;
		this.connected = false;
		
		while( this.running ) {
			reset();
			
			if( useBluetooth ) {
				System.out.println("Waiting on Bluetooth connection...");
				this.connection = Bluetooth.waitForConnection();
			} else {
				System.out.println("Waiting on USB connection...");
				this.connection = USB.waitForConnection();
			}
			
			this.connected   = true;
			this.inputStream = this.connection.openDataInputStream();
			
			System.out.println("Connected. Entering connected loop.");
			while( this.connected && this.running ) {
				try {
					if( this.inputStream.available() > 0 || !this.isKicking() ) {
						byte op = this.inputStream.readByte();
						switch( op ) {
						case OP_KICK:
							if( !this.isKicking() )
								this.gotoNextKickingPhase();
							break;
						case OP_SHUTDOWN:
							this.running = false;
							break;
						case OP_SPEEDS:
							float leftSpeed = this.inputStream.readFloat();
							float rightSpeed = this.inputStream.readFloat();
							if( !isBackingOff() ) {
								leftSpeed  *= 180.0 / Math.PI;
								rightSpeed *= 180.0 / Math.PI;
								this.setWheelSpeed( 0, leftSpeed );
								this.setWheelSpeed( 1, rightSpeed );
							}
							break;
						default:
							System.err.println("Wrong op-code! Disconnecting...");
							this.connected = false;
							Sound.buzz();
							break;
						}
					}
					
					this.handleKicking();
					Thread.yield();
					
				} catch( IOException e ) {
					System.out.println("Lost connection.");
					this.connected = false;
				}
				
				if( touchSensors[ 0 ].isPressed() || touchSensors[ 1 ].isPressed() ) {
					backOffEndsAt = System.currentTimeMillis() + 100;
				}
			
				if( isBackingOff() ) {
					float maxSpeed = getWheelMaxSpeed();
					this.setWheelSpeed( 0, -0.95f * maxSpeed );
					this.setWheelSpeed( 1, -0.95f * maxSpeed );
				}
			}
			
			System.out.println("Cleaning up...");
			try {
				this.inputStream.close();
				this.connection.close();
			} catch( Exception e ) {
				// suppress exceptions at this point
			}
		}
		
		System.out.println("Shutting down.");
	}
	
	private void reset() {
		Motor.A.resetTachoCount(); Motor.B.resetTachoCount(); Motor.C.resetTachoCount();
		Motor.A.flt(); Motor.B.flt(); Motor.C.flt();
		this.wheelSpeeds[ 0 ] = this.wheelSpeeds[ 1 ] = 0.0f;
		this.kickingPhase     = KickingPhase.NOT_KICKING;
		this.kickingEndsAt    = -1;
		this.backOffEndsAt    = -1;
	}
	
	private void setWheelSpeed( int wheel, float newSpeed ) {
		float oldSpeed = this.wheelSpeeds[ wheel ];
		float maxSpeed = getWheelMaxSpeed();
		
		if( newSpeed == oldSpeed  )
			return;
		
		NXTRegulatedMotor motor = this.wheelMotors[ wheel ];
		
		if( newSpeed > 0.0f ) {
			motor.setSpeed( Math.min( maxSpeed, newSpeed ) );
			if( oldSpeed <= 0.0f )
				motor.forward();
		} else if( newSpeed < 0.0f ) {
			motor.setSpeed( Math.min( maxSpeed, -newSpeed ) );
			if( oldSpeed >= 0.0f )
				motor.backward();
		} else {
			motor.setSpeed( 0.0f );
			motor.stop( true );
		}
		
		this.wheelSpeeds[ wheel ] = newSpeed;
	}
		
	private void handleKicking() {
		if( this.justEnteredKickingPhase() ) {
			switch( this.kickingPhase ) {
			case NOT_KICKING: break;
			case UP:
				kickerMotor.setSpeed( kickerMotor.getMaxSpeed()*100 );
				kickerMotor.rotate( 50, true );
				setKickPhaseDuration( 300 );
				break;
			case BACK:
				kickerMotor.rotate( -50, true );
				setKickPhaseDuration( 300 );
				break;
			}
		} else {
			long currentTime = System.currentTimeMillis();
			if( currentTime > this.kickingEndsAt )
				gotoNextKickingPhase();
		}
	}
	
	private boolean isKicking() {
		return this.kickingPhase != KickingPhase.NOT_KICKING;
	}
	
	private void setKickPhaseDuration( long duration ) {
		this.kickingEndsAt = System.currentTimeMillis() + duration;
	}
	
	private void gotoNextKickingPhase() {
		this.kickingPhase = KickingPhase.values()[ (this.kickingPhase.ordinal() + 1) % KickingPhase.values().length ];
		this.kickingEndsAt = -1;
		
		System.out.println( this.kickingPhase.toString() );
	}
	
	private boolean justEnteredKickingPhase() {
		return this.kickingEndsAt == -1;
	}
	
	public static void main( String[] args ) {
		(new NxtClient()).run();
	}
	
}