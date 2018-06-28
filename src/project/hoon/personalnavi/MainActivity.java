package project.hoon.personalnavi;

import java.io.File;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

public class MainActivity extends Activity {
	private static final String TAG = "MainActivity";
	private static final int RATE = SensorManager.SENSOR_DELAY_GAME;

	private SensorManager sensorManager;

	private AccelerationEventListener accEvent;
	private GyroscopeEventListener gyroEvent;
	private MagneticfieldEventListener magEvent;

	public boolean isRecord = false;
	public boolean isStart = false;

	private Button startBTN;
	
	public TextView magAzimuth;
	
	public long startTime;
	
	public MapView mapView;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		
		mapView = (MapView)findViewById(R.id.pns);
		//magAzimuth = (TextView)findViewById(R.id.magAzimuth);

		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

		accEvent = new AccelerationEventListener(this);
		magEvent = new MagneticfieldEventListener(this, accEvent, gyroEvent);
		gyroEvent = new GyroscopeEventListener(this, magEvent);

		sensorManager
				.registerListener(accEvent, sensorManager
						.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), RATE);
		sensorManager.registerListener(gyroEvent,
				sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), RATE);
		sensorManager.registerListener(magEvent,
				sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
				RATE);

		startBTN = (Button) findViewById(R.id.startBTN);

		startBTN.setOnClickListener(new Button.OnClickListener() {
			@Override
			public void onClick(View v) {
				isStart = true;
				startTime = SystemClock.uptimeMillis();
			}

		});

	}

	@Override
	protected void onPause() {
		sensorManager.unregisterListener(accEvent);
		sensorManager.unregisterListener(gyroEvent);
		sensorManager.unregisterListener(magEvent);

		super.onPause();
	}

	@Override
	protected void onResume() {
		sensorManager
				.registerListener(accEvent, sensorManager
						.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), RATE);
		sensorManager.registerListener(gyroEvent,
				sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), RATE);
		sensorManager.registerListener(magEvent,
				sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
				RATE);

		super.onResume();
	}

	@Override
	protected void onStop() {
		// TODO Auto-generated method stub
		super.onStop();
	}
	
	public void resetStartTime(){
		startTime = SystemClock.uptimeMillis();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		menu.add(0, 1, 0, "Start Recording");
		menu.add(0, 2, 0, "Stop Recording");
		// menu.add(0, 3, 0, "Init");

		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) {
		case 1:
			if (isRecord == false) {
				startRecordingData();
				isRecord = true;
			}
			break;
		case 2:
			if (isRecord == true) {
				stopRecordingData();
				isRecord = false;
			}
			break;
		case 3:
			break;
		default:
			break;
		}

		return true;
	}

	private void startRecordingData() {
		Log.d(TAG, "Start Recording Sensors Data");

		File accelerometerDataFile = new File(getExternalCacheDir(),
				"accelerometer.csv");
		File gyroscopeDataFile = new File(getExternalCacheDir(),
				"gyroscope.csv");
		File magneticfieldDataFile = new File(getExternalCacheDir(),
				"magneticfield.csv");
		File personalNaviFile = new File(getExternalCacheDir(), "personalnavi.csv");

		accEvent.setPrintWriter(accelerometerDataFile);
		gyroEvent.setPrintWriter(gyroscopeDataFile);
		magEvent.setPrintWriter(magneticfieldDataFile);
		mapView.setPrintWriter(personalNaviFile);
	}

	private void stopRecordingData() {
		Log.d(TAG, "Stop Recording Sensors Data");

		accEvent.recordStop();
		gyroEvent.recordStop();
		magEvent.recordStop();
		mapView.recordStop();

	}
}
