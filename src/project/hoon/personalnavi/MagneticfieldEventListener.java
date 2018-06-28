package project.hoon.personalnavi;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.SystemClock;
import android.util.Log;

public class MagneticfieldEventListener implements SensorEventListener {
	private static final String TAG = "MagneticfieldEventListener";
	private static final String CSV_HEADER = "M_X Axis, M_Y Axis, M_Z Axis, M_Init, M_Azimuth, M_Time";
	private static final char CSV_DELIM = ',';

	private MainActivity mainActivity;
	private AccelerationEventListener accEvent;
	private GyroscopeEventListener gyroEvent;

	private PrintWriter printWriter;

	private long startTime;
	private float[] values;
	private float[] rotationMatrix;
	private float[] orientationValues;
	private float[] initDegree;
	private float[] oriDegree;
	private float initAzimuth;
	private float azimuth;
	
	private float initRadAzimuth;
	private float RadAzimuth;

	public boolean checkOrient = false;

	// private Handler mHandler;

	public MagneticfieldEventListener(MainActivity mainActivity,
			AccelerationEventListener accEvent, GyroscopeEventListener gyroEvent) {

		this.mainActivity = mainActivity;
		this.accEvent = accEvent;
		this.gyroEvent = gyroEvent;
		// mHandler = new Handler();

		startTime = SystemClock.uptimeMillis();
		
		values = new float[3];
		rotationMatrix = new float[16];
		orientationValues = new float[3];
		initDegree = new float[3];
		oriDegree = new float[3];
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		values = event.values.clone();

		if (gyroEvent.Norm == true && checkOrient == false)
			
			setOrientation();
		if (checkOrient == true)
			calcAzimuth();

		if (mainActivity.isRecord == true)
			writeSensorEvent(event.timestamp);
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		Log.d(TAG, "onAccuracyChanged");

	}

	public void setOrientation() {
		float[] acc = accEvent.getFilteredValues();

		SensorManager.getRotationMatrix(rotationMatrix, null, acc, values);
		SensorManager.getOrientation(rotationMatrix, orientationValues);

		setInitAzimuth();

		checkOrient = true;
	}

	private void setInitAzimuth() {
		initRadAzimuth = orientationValues[0];
		initAzimuth = (float) Math.toDegrees(orientationValues[0]);

		Log.e(TAG, "" + initAzimuth);
		gyroEvent.setInitAzimuth(initAzimuth);
		gyroEvent.kalmanYaw.setAngle(initRadAzimuth);
		gyroEvent.kalAngleZ = initRadAzimuth;
		gyroEvent.compZ = initRadAzimuth;
		gyroEvent.compDeg = initAzimuth;
	}

	private void calcAzimuth() {
		float[] acc = accEvent.getFilteredValues();

		SensorManager.getRotationMatrix(rotationMatrix, null, acc, values);
		SensorManager.getOrientation(rotationMatrix, orientationValues);

		RadAzimuth = (float) orientationValues[0];
		azimuth = (float) Math.toDegrees(orientationValues[0]);
		
		mainActivity.mapView.setAngle(azimuth);
	}

	public float getInitAzimuth() {
		return initAzimuth;
	}

	public float getAzimuth() {
		return azimuth;
	}
	
	public float getRadAzimuth(){
		return RadAzimuth;
	}

	public float[] getValues() {
		return values;
	}

	public void setPrintWriter(File dataFile) {
		try {
			printWriter = new PrintWriter(new BufferedWriter(new FileWriter(
					dataFile)));

			printWriter.println(CSV_HEADER);
		} catch (IOException e) {
			Log.e(TAG, "Could not open CSV file(s)", e);
		}
	}

	public void recordStop() {
		if (printWriter != null) {
			printWriter.close();
		}

		if (printWriter.checkError())
			Log.e(TAG, "Error closing writer");
	}

	private void writeSensorEvent(long eventTime) {
		if (printWriter != null) {
			StringBuffer sb = new StringBuffer().append(values[0])
					.append(CSV_DELIM).append(values[1]).append(CSV_DELIM)
					.append(values[2]).append(CSV_DELIM).append(initAzimuth)
					.append(CSV_DELIM).append(azimuth).append(CSV_DELIM)
					.append((eventTime / 1000000) - startTime);

			printWriter.println(sb.toString());

			if (printWriter.checkError())
				Log.e(TAG, "Error writing sensor event data");
		}
	}
}
