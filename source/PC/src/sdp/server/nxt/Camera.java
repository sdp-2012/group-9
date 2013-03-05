package sdp.server.nxt;

import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;
import java.util.List;

import au.edu.jcu.v4l4j.Control;
import au.edu.jcu.v4l4j.FrameGrabber;
import au.edu.jcu.v4l4j.V4L4JConstants;
import au.edu.jcu.v4l4j.VideoDevice;
import au.edu.jcu.v4l4j.VideoFrame;
import au.edu.jcu.v4l4j.exceptions.V4L4JException;

import java.io.IOException;

/** This class wraps communication with the camera, abstracting away the V4L4J details. It provides
 * 	methods for changing properties like brightness or contrast and provides a method for grabbing
 *  the latest class
 *  
 * @author Cristian Cobzarenco.
 *
 */
public class Camera {
	private boolean       cameraConnected = false;
	private VideoDevice   videoDevice;
	private FrameGrabber  frameGrabber;
	private VideoFrame    frame;
	
	// FIXME: Make this into a c-tor argument, with the default set to this.
	public static final String videoDeviceName = "/dev/video0";
	
	/** Set the camera's saturation.
	 * 
	 * @param newSaturation The desired value for the camera's saturation, in the range 0-127.
	 */
	public void setSaturation( int newSaturation ) {
		setProperty( "Saturation", newSaturation );
	}
	
	/** Set the camera's contrast.
	 * 
	 * @param newContrast The desired value for the camera's contrast, in the range 0-127.
	 */
	public void setContrast( int newContrast ) {
		setProperty( "Contrast", newContrast );
	}

	/** Set the camera's brightness.
	 * 
	 * @param newBrightness The desired value for the camera's brightness, in the range 0-255.
	 */
	public void setBrightness( int newBrightness ) {
		setProperty( "Brightness", newBrightness );
	}
	
	/** Set the camera's hue.
	 * 
	 * @param newHue The desired hue value from -128 up to 127.
	 */
	public void setHue( int newHue ) {
		setProperty( "Hue", newHue );
	}
	
	/** Get the camera's current saturation value (0-127). */
	public int getSaturation() {
		try {
			return getPropertyValue( "Saturation" );
		} catch (IOException e) {
			System.err.println("Camera: Could not get saturation from camera.");
			return 63;
		}
	}
	
	/** Get the camera's current contrast value (0-127). */
	public int getContrast() {
		try {
			return getPropertyValue( "Contrast" );
		} catch (IOException e) {
			System.err.println("Camera: Could not get contrast from camera.");
			return 63;
		}
	}
	
	/** Get the camera's current brightness value (0-255). */
	public int getBrightness() {
		try {
			return getPropertyValue( "Brightness" );
		} catch (IOException e) {
			System.err.println("Camera: Could not get brightness from camera.");
			return 127;
		}
	}
	
	/** Get the camera's current hue value (-128-127). */
	public int getHue() {
		try {
			return getPropertyValue( "Hue" );
		} catch (IOException e) {
			System.err.println("Camera: Could not get hue from camera.");
			return 0;
		}
	}
	
	/** Set the value of a custom camera property.
	 * 
	 * @param propertyName The name of the custom camera property.
	 * @param value The desired value for this property
	 * @return true if the value could be set, false otherwise.
	 */
	public boolean setProperty( String propertyName, int value ) {
		int status = 2;
		List<Control> controls = null;
		try {
			controls = videoDevice.getControlList().getList();
			for(Control c: controls) {
				if( c.getName().equals(propertyName) ) {
					c.setValue( value );
					status = 0;
					break;
				}
			}
		 } catch( V4L4JException e ) {
			 e.printStackTrace();
			 System.err.println("Camera: Could not set camera property '" + propertyName + "' to " + value);
			 status = 1;
		 } finally {
			 if( controls != null )
				 videoDevice.releaseControlList();
		 }
		 
		 if( status == 2 )
			 System.err.println("Camera: Could not find camera property '" + propertyName + "'");
		 
		 return status == 0;
	}
	
	/** Retrieves the value of a custom camera property.
	 * 
	 * @param propertyName The name of the custom camera property.
	 * @return The value of the property
	 * @throws IOException If the property doesn't exist.
	 */
	public int getPropertyValue( String propertyName ) throws IOException {
		List<Control> controls = null;
		try {
			controls = videoDevice.getControlList().getList();
			for(Control c: controls) {
				if( c.getName().equals(propertyName) )
					return c.getValue();
			}
		 } catch( V4L4JException e ) {
			 throw new IOException( e );
		 } finally {
			 if( controls != null )
				 videoDevice.releaseControlList();
		 }
		 
		 throw new IOException("Could not find camera property '" + propertyName + "'");
	}
	
	/** Retrieves the range of a custom camera property.
	 * 
	 * @param propertyName The name of the custom camera property.
	 * @return The range of the property ([min, max]).
	 * @throws IOException if the property doesn't exist.
	 */
	public int[] getPropertyRange( String propertyName ) throws IOException {
		List<Control> controls = videoDevice.getControlList().getList();
		for(Control c: controls) {
			if( c.getName().equals(propertyName) ) {
				int[] range = {c.getMinValue(), c.getMaxValue()};
				videoDevice.releaseControlList();
				return range;
			}
		}
		videoDevice.releaseControlList();
		throw new IOException("Could not find camera property '" + propertyName + "'");
	}
	
	/** Update method for the camera, should be called every frame. It grabs the newest frame from
	 * the camera, which can then be retrieved from getFrame(). If not already connected it attempts
	 * to connect to the camera.
	 */
	@SuppressWarnings("deprecation")
	public void update() {
		assert isConnected();
		boolean frameGrabbingFailed = false;
		do {
			if( frame != null )
				frame.recycle();
			
			try {
				frame = frameGrabber.getVideoFrame();
			} catch( V4L4JException e ) {
				frameGrabbingFailed = true;
				
				String message = e.getMessage();
				disconnect();
				while( !isConnected() ) {
					System.err.println("Camera: Disconnected (" + message + "). Reconnecting in 1s..." );
					
					try {
						Thread.sleep(1000);
					} catch( InterruptedException e3 ) {}
					
					try {
						connect();
					} catch( IOException e2 ) {
						message = e.getMessage();
					}
				}
				System.out.println("Camera: Reconnected.");
			}
			
		} while( frameGrabbingFailed );
	}
	
	/** Get the newest frame from the camera (from when update() was last called).
	 * 
	 * @return A buffered image containing the new frame.
	 */
	public BufferedImage getCameraImage() {
		return copyBufferedImageHack( frame.getBufferedImage() );
	}
	
	/** Check whether the camera is connected.
	 * 
	 * @return true if the camera is connected, false otherwise.
	 */
	public boolean isConnected() {
		return cameraConnected;
	}
	
	/** Attempt to establish a connection to the camera.
	 * 
	 * @throws ConnectionFailed If the connection could not be established.
	 */
	public void connect() throws IOException {
		try {
			System.out.println("Camera: Opening video device '" + videoDeviceName + "'...");
			videoDevice  = new VideoDevice( videoDeviceName );
			
			// Dump a list of the camera's properties and their ranges onto stdout.
			System.out.println("Camera: Device opened. Adjustable properties are:");
			List<Control> controls = null;
			try {
				controls = videoDevice.getControlList().getList();
				for(Control c: controls)
					System.out.println("Camera: " + c.getName() + " = " + c.getValue() + " in " + c.getMinValue() + "-" + c.getMaxValue());
			 } catch( V4L4JException e ) {
				 throw new IOException(e);
			 } finally {
				 if( controls != null )
					 videoDevice.releaseControlList();
			 }
			
			System.out.println("Camera: Creating frame grabber...");
			frameGrabber = videoDevice.getBGRFrameGrabber(0, 0, 0, V4L4JConstants.STANDARD_PAL);
			System.out.println("Camera: Feed resolution: " + frameGrabber.getWidth() + "x" + frameGrabber.getHeight());
			System.out.println("Camera: Starting capture");
			frameGrabber.startCapture();
		} catch( V4L4JException e ) {
			if( frameGrabber != null ) {
				videoDevice.releaseFrameGrabber();
				frameGrabber = null;
			}
			
			if( videoDevice != null ) {
				videoDevice.release();
				videoDevice = null;
			}
			
			throw new IOException( e );
		}
		System.out.println("Camera: Camera connected.");
		cameraConnected = true;
		
		update();
	}
	
	/** Terminates the connection to the camera, if connected. Does nothing if not. */
	public void disconnect() {
		System.out.println("Camera: Disconnecting camera.");
		if( frameGrabber != null ) {
			frameGrabber.stopCapture();
			videoDevice.releaseFrameGrabber();
			frameGrabber = null;
		}
		
		if( videoDevice != null ) {
			videoDevice.release();
			videoDevice = null;
		}
		System.out.println("Camera: Camera disconnected.");
		cameraConnected = false;
	}
	
	// This hack performs an actual, deep-copy, of a BufferedImage - for some reason the vision doesn't work otherwise.
	private static BufferedImage copyBufferedImageHack( BufferedImage image ) {
		ColorSpace cs = ColorSpace.getInstance(ColorSpace.CS_sRGB);
		ColorModel cm = new ComponentColorModel(cs, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
		BufferedImage copy = new BufferedImage(cm, image.copyData(null), false, null);
		return copy;
	}
}
