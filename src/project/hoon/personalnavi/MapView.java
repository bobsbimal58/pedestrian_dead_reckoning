package project.hoon.personalnavi;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.SystemClock;
import android.text.InputFilter.LengthFilter;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Display;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import android.view.SurfaceView;
import android.view.WindowManager;

public class MapView extends SurfaceView implements Callback {
	private static final String TAG = "MapView";
	private static final String CSV_HEADER = "Step, Length, Azimuth, compRad, compDeg, kalman, gyro, Time";
	private static final char CSV_DELIM = ',';

	private MapThread mThread;
	private SurfaceHolder mHolder;
	private Context mContext;

	private PrintWriter printWriter;
	private long startTime;

	private float lx, ly;

	public MapView(Context context, AttributeSet attrs) {
		super(context, attrs);
		SurfaceHolder holder = getHolder();
		holder.addCallback(this);

		startTime = SystemClock.uptimeMillis();

		mHolder = holder;
		mContext = context;
		mThread = new MapThread(holder, context);

		lx = 0.0f;
		ly = 0.0f;
	}

	public void setAngle(float _ang) {
		mThread.setAzimuth(_ang);
	}

	public void setCompYawRad(float _ang) {
		mThread.setCompYawRad(_ang);
	}

	public void setCompYawDeg(float _ang) {
		mThread.setCompYawDeg(_ang);
	}

	public void setKalmanYaw(float _ang) {
		mThread.setKalmanYaw(_ang);
	}

	public void setGyroYaw(float _ang) {
		mThread.setGyroYaw(_ang);
	}

	public void setStepCnt(int _step) {
		mThread.setStepCount(_step);
	}

	public void setStepLength(float _length) {
		mThread.setStepLength(_length);
	}

	public void setDetectedTime(long _time) {
		mThread.setDetectedTime(_time);
	}

	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		mThread.start();
	}

	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int width,
			int height) {

	}

	@Override
	public void surfaceDestroyed(SurfaceHolder holder) {
		boolean done = true;
		while (done) {
			try {
				mThread.join();
				done = false;
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public void StopMapThread() {
		mThread.StopThread();
	}

	public void PauseMapThread() {
		mThread.PauseNResume(true);
	}

	public void ResumeMapThread() {
		mThread.PauseNResume(false);
	}

	public void restartMapThread() {
		mThread.StopThread();

		mThread = null;
		mThread = new MapThread(mHolder, mContext);
		mThread.start();
	}

	class MapThread extends Thread {
		private SurfaceHolder mHolder;
		private Context mContext;

		private int width, height; // Display width/ height
		private int cw, ch; // Display center

		private boolean canRun = true;
		private boolean isWait = false;

		private Bitmap imgBack;
		private Bitmap imgMatrix;
		private Bitmap imgMap;
		private int mw, mh;

		private Bitmap imgArrow;
		private int aw, ah;

		private float _x, _y;
		private float x, y;

		private float cen_x, cen_y;

		private float aa, bb;

		private Paint paint;

		private float azimuth;
		private float stepLength;
		private int stepCount;
		private int preStep;
		private long stepTime;

		private float compYawDeg;
		private float compYawRad;
		private float kalmanYaw;
		private float gyroYaw;

		public MapThread(SurfaceHolder holder, Context context) {
			mHolder = holder;
			mContext = context;

			Display display = ((WindowManager) context
					.getSystemService(Context.WINDOW_SERVICE))
					.getDefaultDisplay();

			paint = new Paint();

			width = display.getWidth();
			height = display.getHeight();

			cw = (int) width / 2;
			ch = (int) height / 2;

			cen_x = cw;
			cen_y = ch;

			imgBack = BitmapFactory.decodeResource(getResources(),
					R.drawable.back);
			imgBack = Bitmap.createScaledBitmap(imgBack, width, height - 230,
					false);

			paint.setTextSize(25);
			paint.setColor(Color.BLUE);
			paint.setAntiAlias(true);

		}

		public void run() {
			Canvas canvas = null;
			while (canRun) {
				canvas = mHolder.lockCanvas();
				try {
					synchronized (mHolder) {
						DrawCharacters(canvas);
					}
				} finally {
					if (canvas != null)
						mHolder.unlockCanvasAndPost(canvas);
				}

				synchronized (this) {
					if (isWait) {
						try {
							wait();
						} catch (Exception e) {
							e.printStackTrace();
						}
					}
				}
			}
		}

		public void StopThread() {
			canRun = false;
			synchronized (this) {
				this.notify();
			}
		}

		public void PauseNResume(boolean wait) {
			isWait = wait;
			synchronized (this) {
				this.notify();
			}
		}

		public void DrawCharacters(Canvas canvas) {
			canvas.drawBitmap(imgBack, 0, 0, null);
			Paint pnt = new Paint();
			pnt.setColor(Color.RED);
			canvas.drawCircle(100, 90, 50, pnt);
			
			canvas.drawText("Step    : " + stepCount, 10, 30, paint);
			canvas.drawText("Length  : " + stepLength, 10, 60, paint);
			canvas.drawText("Azimuth : " + azimuth, 10, 90, paint);
		}

		public void setAzimuth(float _ang) {
			azimuth = _ang;
		}

		public void setCompYawRad(float _ang) {
			compYawRad = _ang;
		}

		public void setCompYawDeg(float _ang) {
			compYawDeg = _ang;
		}

		public void setKalmanYaw(float _ang) {
			kalmanYaw = _ang;
		}

		public void setGyroYaw(float _ang) {
			gyroYaw = _ang;
		}

		public void setStepCount(int _step) {
			stepCount = _step;
		}

		public void setStepLength(float _length) {
			stepLength = _length;

			synchronized (this) {
				float _azimuth = Math.abs(azimuth);
				float _radAzimuth = (float) Math.toRadians(_azimuth);

				if (azimuth < 0) {

				} else {

				}

				writeLocationEvent(stepCount, stepLength, azimuth, compYawRad,
						compYawDeg, kalmanYaw, gyroYaw, stepTime);
			}
		}

		public void setDetectedTime(long _time) {
			stepTime = _time;
		}
	}

	public void setPrintWriter(File personalNaviFile) {
		try {
			printWriter = new PrintWriter(new BufferedWriter(new FileWriter(
					personalNaviFile)));
			printWriter.println(CSV_HEADER);
		} catch (IOException e) {
			Log.e(TAG, "Could not open CSV file(s)", e);
		}
	}

	public void recordStop() {
		if (printWriter != null) {
			printWriter.close();
		}

		if (printWriter.checkError()) {
			Log.e(TAG, "Error closing writer");
		}
	}

	private void writeLocationEvent(int step, float length, float ang,
			float compRad, float compDeg, float kalman, float gyro,
			long eventTime) {
		if (printWriter != null) {
			StringBuffer sb = new StringBuffer().append(step).append(CSV_DELIM)
					.append(length).append(CSV_DELIM).append(ang)
					.append(CSV_DELIM).append(compRad).append(CSV_DELIM)
					.append(compDeg).append(CSV_DELIM).append(kalman)
					.append(CSV_DELIM).append(gyro).append(CSV_DELIM)
					.append(eventTime);

			printWriter.println(sb.toString());

			if (printWriter.checkError())
				Log.e(TAG, "Error writing Location Data");
		}
	}
}
