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
import android.provider.Settings.System;
import android.util.Log;

public class GyroscopeEventListener implements SensorEventListener {
	private static final String TAG = "GyroscopeEventListener";
	private static final float NS2S = 1.0f / 1000000000.0f;
	private static final String CSV_HEADER = "G_X Axis, G_Y Axis, G_Z Axis, G_N_X, G_N_Y, G_N_Z, G_InitAzimuth, G_CurAzimuth, G_N_Cur, G_Position, G_N_P, G_Kalman, G_Comp, G_Time";
	private static final char CSV_DELIM = ',';
	
	private final float[] deltaRotationVector = new float[4];
	private float[] gyroscopeOrientation = new float[3];
	
	private static final float FILTER_COEFFICIENT = 0.95f;

	private MainActivity mainActivity;
	private MagneticfieldEventListener magEvent;

	private PrintWriter printWriter;

	private long startTime;
	private long timestamp;
	private long N_timestamp;

	private float[] values;
	private float[] radAngle;
	private float[] N_radAngle;
	private float[] degree;
	private float[] N_degree;

	private static double initAzimuth;
	private float curAzimuth;
	private float N_curAzimuth;
	private float filtered;

	public static KalmanFilter kalmanYaw = new KalmanFilter();

	public static boolean Norm = false;

	// average Filter value
	private int k = 1;
	private float[] prevAvg = new float[3];
	private float alpha = 0;
	private float[] avg = new float[3];

	public static float kalAngleZ = 0.0f;
	public static float compZ = 0.0f;
	public static float compYaw = 0.0f;
	public static float compYawDeg = 0.0f;
	public static float compDeg = 0.0f;
	private float kalYaw = 0.0f;

	public GyroscopeEventListener(MainActivity mainActivity,
			MagneticfieldEventListener magEvent) {
		this.mainActivity = mainActivity;
		this.magEvent = magEvent;

		startTime = SystemClock.uptimeMillis();

		values = new float[3];
		radAngle = new float[3];
		degree = new float[3];
		N_degree = new float[3];
		N_radAngle = new float[3];
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		values = event.values.clone();
		
		if (mainActivity.isStart == true && Norm == false) {
			long curTime = SystemClock.uptimeMillis();

			checkNorm(curTime);
		}

		if (Norm == true)
			calRadAzimuth(values, event.timestamp);

		if (mainActivity.isRecord)
			writeSensorEvent(event.timestamp);

	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub

	}

	private void checkNorm(long curTime) {
		if (curTime - mainActivity.startTime <= 3000) {
			averageFilter(values);
		} else if (curTime - mainActivity.startTime > 3000) {
			Norm = true;

			Log.d(TAG, "avg[0] : " + avg[0] + ", avg[1] : " + avg[1]
					+ ", avg[2] : " + avg[2]);
		}
	}
	
	private  void compFusedAzimuth(float[] value, long eventTime){
		final float dT = (eventTime - timestamp) * NS2S;
		
		float axisX = value[0];
		float axisY = value[1];
		float axisZ = value[2];
		
		float omegaMagnitude = (float)Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
		
		float thetaOverTwo = omegaMagnitude * dT / 2.0f;
		
		float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
		float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
		
		deltaRotationVector[0] = sinThetaOverTwo * axisX;
		deltaRotationVector[1] = sinThetaOverTwo * axisY;
		deltaRotationVector[2] = sinThetaOverTwo * axisZ;
		
		float[] deltaRotationMatrix = new float[9];
		
		SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
		
		//currentRotationMatrix = matrixMultiplication(currentRotationMatrix, deltaRotationMatrix);
		
		//SensorManager.getOrientation(currentRotationMatrix, gyroscopeOrientation);
	}

	private void calRadAzimuth(float[] value, long eventTime) {
		final float magAzimuth = magEvent.getRadAzimuth();

		if (timestamp != 0) {
			final float dT = (eventTime - timestamp) * NS2S;
			radAngle[0] += value[0] * dT;
			radAngle[1] += value[1] * dT;
			radAngle[2] += value[2] * dT;

			N_radAngle[0] = N_radAngle[0] + (value[0] - avg[0]) * dT;
			N_radAngle[1] = N_radAngle[1] + (value[1] - avg[1]) * dT;
			N_radAngle[2] = N_radAngle[2] + (value[2] - avg[2]) * dT;

			kalAngleZ = kalmanYaw.getAngle(magAzimuth, -value[2], dT);
			complementaryFilter(magAzimuth, value[2], dT);

			kalYaw = (float) Math.toDegrees(kalAngleZ);
			
			mainActivity.mapView.setKalmanYaw(kalYaw);
			
			
		}

		calDegAzimuth();
		calDegAzimuthWithNorm();
		timestamp = eventTime;
	}

	private void complementaryFilter(float magAzimuth, float value, float dT) {
		compZ = FILTER_COEFFICIENT * (compZ + (-value) * dT) + (1 - FILTER_COEFFICIENT) * magAzimuth;
		compYawDeg = FILTER_COEFFICIENT * compDeg + (1 - FILTER_COEFFICIENT) * magEvent.getAzimuth();

		compYaw = (float) Math.toDegrees(compZ);
		
		mainActivity.mapView.setCompYawDeg(compYawDeg);
		mainActivity.mapView.setCompYawRad(compYaw);
	}

	private void averageFilter(float[] values) {
		alpha = (k - 1) / k;

		avg[0] = alpha * prevAvg[0] + (1 - alpha) * values[0];
		avg[1] = alpha * prevAvg[1] + (1 - alpha) * values[1];
		avg[2] = alpha * prevAvg[2] + (1 - alpha) * values[2];

		prevAvg[0] = avg[0];
		prevAvg[1] = avg[1];
		prevAvg[2] = avg[2];
		k = k + 1;
	}

	private void calDegAzimuth() {
		degree[0] = (float) Math.toDegrees(radAngle[0]);
		degree[1] = (float) Math.toDegrees(radAngle[1]);
		degree[2] = (float) Math.toDegrees(radAngle[2]);

		curAzimuth = degree[2];
		compDeg = (float) (initAzimuth - curAzimuth);
		
		mainActivity.mapView.setGyroYaw(compDeg);
	}

	private void calDegAzimuthWithNorm() {
		N_degree[0] = (float) Math.toDegrees(N_radAngle[0]);
		N_degree[1] = (float) Math.toDegrees(N_radAngle[1]);
		N_degree[2] = (float) Math.toDegrees(N_radAngle[2]);

		N_curAzimuth = N_degree[2];
	}

	public double getCurrentAzimuth() {
		return curAzimuth;
	}

	public static void setInitAzimuth(float azimuth) {
		initAzimuth = azimuth;

		Log.d(TAG, "" + initAzimuth);
	}

	public float[] getDegree() {
		return degree;
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
		if (printWriter != null)
			printWriter.close();

		if (printWriter.checkError())
			Log.e(TAG, "Error closing writer");
	}

	private void writeSensorEvent(long eventTime) {
		if (printWriter != null) {
			StringBuffer sb = new StringBuffer().append(values[0])
					.append(CSV_DELIM).append(values[1]).append(CSV_DELIM)
					.append(values[2]).append(CSV_DELIM)
					.append(values[0] - avg[0]).append(CSV_DELIM)
					.append(values[1] - avg[1]).append(CSV_DELIM)
					.append(values[2] - avg[2]).append(CSV_DELIM)
					.append(initAzimuth).append(CSV_DELIM).append(curAzimuth)
					.append(CSV_DELIM).append(N_curAzimuth).append(CSV_DELIM)
					.append(initAzimuth - curAzimuth).append(CSV_DELIM)
					.append(initAzimuth - N_curAzimuth).append(CSV_DELIM)
					.append(kalYaw).append(CSV_DELIM).append(compYaw)
					.append(CSV_DELIM).append(compYawDeg).append(CSV_DELIM)
					.append((eventTime / 1000000) - startTime);

			printWriter.println(sb.toString());
		}

		if (printWriter.checkError())
			Log.e(TAG, "Error writing senor event data");
	}

	private float[] getRotationMatrixFromOrientation(float[] orientation) {
		float[] xM = new float[9];
		float[] yM = new float[9];
		float[] zM = new float[9];

		float sinX = (float) Math.sin(orientation[1]);
		float cosX = (float) Math.cos(orientation[1]);
		float sinY = (float) Math.sin(orientation[2]);
		float cosY = (float) Math.cos(orientation[2]);
		float sinZ = (float) Math.sin(orientation[0]);
		float cosZ = (float) Math.cos(orientation[0]);

		// x축에 대한 회전(Pitch)
		xM[0] = 1.0f;
		xM[1] = 0.0f;
		xM[2] = 0.0f;
		xM[3] = 0.0f;
		xM[4] = cosX;
		xM[5] = sinX;
		xM[6] = 0.0f;
		xM[7] = -sinX;
		xM[8] = cosX;

		// y축에 대한 회전(Roll)
		yM[0] = cosY;
		yM[1] = 0.0f;
		yM[2] = sinY;
		yM[3] = 0.0f;
		yM[4] = 1.0f;
		yM[5] = 0.0f;
		yM[6] = -sinY;
		yM[7] = 0.0f;
		yM[8] = cosY;

		// z측에 대한 회전(Azimuth/Yaw)
		zM[0] = cosZ;
		zM[1] = sinZ;
		zM[2] = 0.0f;
		zM[3] = -sinZ;
		zM[4] = cosZ;
		zM[5] = 0.0f;
		zM[6] = 0.0f;
		zM[7] = 0.0f;
		zM[8] = 1.0f;

		// 행렬곱을 통해 회전 행렬을 산출한다
		// 이때 회전 순서는 y-x-z로 진행한다.
		float[] resultMatrix = matrixMultiplication(xM, yM);
		resultMatrix = matrixMultiplication(zM, resultMatrix);
		return resultMatrix;
	}
	
	private float[] matrixMultiplication(float[] a, float[] b)
	{
		float[] result = new float[9];

		result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
		result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
		result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

		result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
		result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
		result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

		result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
		result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
		result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

		return result;
	}

}
