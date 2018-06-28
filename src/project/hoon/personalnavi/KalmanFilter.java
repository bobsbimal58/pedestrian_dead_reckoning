package project.hoon.personalnavi;

import android.util.Log;

public class KalmanFilter {
	private static float Q_angle = 0.001f;
	private static float Q_bias = 0.003f;
	private static float R_measure = 0.03f;

	private static float angle = 0.0f;
	private static float bias = 0.0f;

	private static float[][] P = new float[2][2];
	private static float rate;
	
	public KalmanFilter() {
		
		P[0][0] = 0.0f;
		P[0][1] = 0.0f;
		P[1][0] = 0.0f;
		P[1][1] = 0.0f;
	}

	public float getAngle(float newAngle, float newRate, float dt) {
		rate = newRate - bias;
		angle += dt * rate;
		
		P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
		P[0][1] -= dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_bias * dt;
		
		float S = P[0][0] + R_measure;
		
		float [] K = new float[2];
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;
		
		float y = newAngle - angle;
		angle += K[0] * y;
		bias += K[1] * y;
		
		float P00_temp = P[0][0];
		float P01_temp = P[0][1];
		
		P[0][0] -= K[0] * P00_temp;
		P[0][1] -= K[0] * P01_temp;
		P[1][0] -= K[0] * P00_temp;
		P[1][1] -= K[1] * P01_temp;
		
		return angle;
	}
	
	public void setAngle(float angle){
		this.angle = angle;
	}
	
	public float getRate(){
		return this.rate;
	}
	
	public void setQangle(float Q_angle){
		this.Q_angle = Q_angle;
	}
	
	public void setQbias(float Q_bias){
		this.Q_bias = Q_bias;
	}
	
	public void setRmeasure(float R_measure){
		this.R_measure = R_measure;
	}
	
	public float getQangle(){
		return this.Q_angle;
	}
	
	public float getQbias(){
		return this.Q_bias;
	}
	
	public float getRmeasure(){
		return this.R_measure;
	}
}
