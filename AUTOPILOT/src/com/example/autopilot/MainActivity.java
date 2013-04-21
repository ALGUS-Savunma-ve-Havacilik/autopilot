package com.example.autopilot;

import java.io.IOException;
import java.net.MalformedURLException;
import java.util.concurrent.ExecutionException;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.widget.TextView;
import android.widget.Toast;
import android.view.Menu;
import android.view.View;

import org.json.JSONException;
import org.json.JSONObject;

public class MainActivity extends Activity implements LocationListener,
		SensorEventListener {
	// Define UI Fields
	private TextView XField, YField, ZField, phiField, thetaField, psiField,
			uField, vField, wField, pField, qField, rField;
	private Handler handler = new Handler();

	// Define Sensors and Location Listener
	public LocationManager locationManager;
	private String provider;

	private SensorManager mSensorManager;
	private Sensor mAccelerometer;
	private Sensor mGyroscope;
	private Sensor mGeomagnetic;
	private Sensor mPressure;
	private Sensor mGravity;

	// Define Class Variables
	float[] mAccel = new float[3];
	float[] mGeoMags = new float[3];
	float[] mGrav = new float[3];
	float[] mOmegaBody = new float[3];
	float[] mPress = new float[3];
	float[] mRotationM = new float[9];
	float[] mOrientation = new float[3];
	float[] pressure_gradient = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float ground_speed, ground_bearing, lat, lon, wind_speed, wind_bearing;
	float p0 = SensorManager.PRESSURE_STANDARD_ATMOSPHERE;
	float GEOHEIGHT = 1608.637939453125f;
	float[] POS0 = new float[3];

	// Initialize Aircraft Dynamics Variables
	float[] position_inertial = { 0, 0, 0 };
	float[] velocity_inertial = { 0, 0, 0 };
	float[] velocity_body = { 0, 0, 0 };
	float[] euler_angles = { 0, 0, 0 };
	float[] omega_body = { 0, 0, 0 };

	float[] ECEF = { 0, 0, 0 };

	float[] aircraft_state = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	private Runnable runnable = new Runnable() {
		@Override
		public void run() {
			setCurrentState();
			updateValues();

			handler.postDelayed(this, 10);
		}
	};

	public void computeOrientation() {

		if (SensorManager.getRotationMatrix(mRotationM, null, mGrav, mGeoMags)) {
			float[] temp = new float[3];
			SensorManager.getRotationMatrix(mRotationM, null, mGrav, mGeoMags);
			// float [][] totRot =
			// Matrix.multiply(Matrix.vectorToArray(mRotationM),
			// Matrix.vectorToArray(mInclinationM));
			SensorManager.getOrientation(mRotationM, mOrientation);

			temp[0] = (float) (mOrientation[2]);
			temp[1] = (float) (mOrientation[1]);
			temp[2] = (float) (mOrientation[0]);

			mOrientation = temp;
		}
	}

	private float findVerticalVelocity() {
		int n = pressure_gradient.length;
		for (int i = 1; i < n; i++) {
			pressure_gradient[i] = pressure_gradient[i - 1];
		}
		pressure_gradient[0] = mPress[0];

		return (SensorManager.getAltitude(p0, pressure_gradient[0]) - SensorManager
				.getAltitude(p0, pressure_gradient[n - 1]));
	}

	private void findXYPosition() {
		float[] delta = new float[3];

		float[][] ROT1 = new float[3][3];
		float[][] ROT2 = new float[3][3];

		updateECEF();

		delta[0] = POS0[0] - ECEF[0];
		delta[1] = POS0[1] - ECEF[1];
		delta[2] = POS0[2] - ECEF[2];

		ROT1[0][0] = (float) Math.cos(lon);
		ROT1[0][1] = (float) Math.sin(lon);
		ROT1[0][2] = 0;

		ROT1[1][0] = (float) -Math.sin(lon);
		ROT1[1][1] = (float) Math.cos(lon);
		ROT1[1][2] = 0;

		ROT1[2][0] = 0;
		ROT1[2][1] = 0;
		ROT1[2][2] = 1;

		ROT2[0][0] = (float) -Math.sin(lat);
		ROT2[0][1] = 0;
		ROT2[0][2] = (float) Math.cos(lat);

		ROT2[1][0] = 0;
		ROT2[1][1] = 1;
		ROT2[1][2] = 0;

		ROT2[2][0] = (float) -Math.cos(lat);
		ROT2[2][1] = 0;
		ROT2[2][2] = (float) -Math.sin(lat);

		position_inertial = Matrix.multiply(Matrix.multiply(ROT2, ROT1), delta);
	}

	/**
	 * Sets the ground state values. Assigns the current pressure to from the
	 * barometric pressure sensor to p0. Sets origin for the North East down
	 * coordinate system in ECEF coordinates to POS0. Gets weather of current
	 * location by getWeather().
	 */
	private void getGroundState() {
		p0 = mPress[0];

		updateECEF();
		System.arraycopy(ECEF, 0, POS0, 0, 3);
		try {
			getWeather();
		} catch (MalformedURLException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Finds the current weather for the present location as set by the GPS
	 * receiver. Assigns values for local wind speed and direction to wind_speed
	 * and wind_bearing.
	 * 
	 * @throws MalformedURLException
	 * @throws IOException
	 */
	private void getWeather() throws MalformedURLException, IOException {
		try {
			String WX = new jsonFromURL().execute(
					"https://api.forecast.io/forecast/61be74bdb1292056b2de9c9be6e94ffe/"
							+ String.valueOf(lat) + "," + String.valueOf(lon)
							+ "?units=si").get();
			try {
				JSONObject weather = new JSONObject(WX);
				wind_speed = (float) weather.getJSONObject("currently")
						.getDouble("windSpeed");
				wind_bearing = (float) weather.getJSONObject("currently")
						.getDouble("windBearing") + 180.0f;
			} catch (JSONException e) {
				e.printStackTrace();
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (ExecutionException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void onAccuracyChanged(Sensor arg0, int arg1) {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see android.app.Activity#onCreate(android.os.Bundle)
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		// Initialize Display Variables
		setContentView(R.layout.activity_main);
		XField = (TextView) findViewById(R.id.X);
		YField = (TextView) findViewById(R.id.Y);
		ZField = (TextView) findViewById(R.id.Z);
		phiField = (TextView) findViewById(R.id.phi);
		thetaField = (TextView) findViewById(R.id.theta);
		psiField = (TextView) findViewById(R.id.psi);
		uField = (TextView) findViewById(R.id.u);
		vField = (TextView) findViewById(R.id.v);
		wField = (TextView) findViewById(R.id.w);
		pField = (TextView) findViewById(R.id.p);
		qField = (TextView) findViewById(R.id.q);
		rField = (TextView) findViewById(R.id.r);

		// Get the location manager
		locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
		// Define Sensor manager, and accelerometer
		mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		mAccelerometer = mSensorManager
				.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
		mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		mGeomagnetic = mSensorManager
				.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
		mPressure = mSensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
		mGravity = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

		provider = LocationManager.GPS_PROVIDER;
		Location location = locationManager.getLastKnownLocation(provider);

		locationManager.requestLocationUpdates(provider, 500, 1, this);

		mSensorManager.registerListener(this, mAccelerometer,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mGyroscope,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mGeomagnetic,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mPressure,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mGravity,
				SensorManager.SENSOR_DELAY_FASTEST);

		if (location != null) {
			onLocationChanged(location);
		} else {
		}

	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	@Override
	public void onLocationChanged(Location location) {
		ground_speed = location.getSpeed();
		ground_bearing = location.getBearing();
		lat = (float) location.getLatitude();
		lon = (float) location.getLongitude();
	}

	/* Remove the locationlistener updates when Activity is paused */
	@Override
	protected void onPause() {
		super.onPause();
		locationManager.removeUpdates(this);
		mSensorManager.unregisterListener(this);
	}

	@Override
	public void onProviderDisabled(String provider) {
		Toast.makeText(this, "Disabled provider " + provider,
				Toast.LENGTH_SHORT).show();
	}

	@Override
	public void onProviderEnabled(String provider) {
		Toast.makeText(this, "Enabled new provider " + provider,
				Toast.LENGTH_SHORT).show();
	}

	/* Request updates at startup */
	@Override
	protected void onResume() {
		super.onResume();
		locationManager.requestLocationUpdates(provider, 100, 1, this);
		mSensorManager.registerListener(this, mAccelerometer,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mGyroscope,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mGeomagnetic,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mPressure,
				SensorManager.SENSOR_DELAY_FASTEST);
		mSensorManager.registerListener(this, mGravity,
				SensorManager.SENSOR_DELAY_FASTEST);
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		switch (event.sensor.getType()) {
		case Sensor.TYPE_LINEAR_ACCELERATION:
			System.arraycopy(event.values, 0, mAccel, 0, 3);
			break;
		case Sensor.TYPE_MAGNETIC_FIELD:
			System.arraycopy(event.values, 0, mGeoMags, 0, 3);
			break;
		case Sensor.TYPE_GRAVITY:
			System.arraycopy(event.values, 0, mGrav, 0, 3);
			break;
		case Sensor.TYPE_GYROSCOPE:
			System.arraycopy(event.values, 0, mOmegaBody, 0, 3);
			break;
		case Sensor.TYPE_PRESSURE:
			System.arraycopy(event.values, 0, mPress, 0, 3);
			break;
		default:
			return;
		}
		computeOrientation();
	}

	@Override
	public void onStatusChanged(String provider, int status, Bundle extras) {
	}

	/**
	 * Sets the current state of the aircraft.
	 * 
	 * Depends On: lat = latitude lon = longitude Sensor Readings
	 * <p>
	 * Assigns: aircraft_state = 12x1 state vector
	 * [position;orientation;velocity_body;omega_body]
	 */
	private void setCurrentState() {
		float[][] FLIPPER_WB = { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } };
		float[][] FLIPPER_EA = { { 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } };

		findXYPosition();
		euler_angles = Matrix.multiply(FLIPPER_EA, mOrientation);
		velocity_inertial[0] = (float) (ground_speed
				* Math.cos(ground_bearing * Math.PI / 180) - wind_speed
				* Math.cos(wind_bearing * Math.PI / 180));
		velocity_inertial[1] = (float) (ground_speed
				* Math.sin(ground_bearing * Math.PI / 180) - wind_speed
				* Math.sin(wind_bearing * Math.PI / 180));
		velocity_inertial[2] = findVerticalVelocity();
		velocity_body = Aircraft.TransformFromInertialToBody(velocity_inertial,
				euler_angles);
		omega_body = Matrix.multiply(FLIPPER_WB, mOmegaBody);

		System.arraycopy(position_inertial, 0, aircraft_state, 0, 3);
		System.arraycopy(euler_angles, 0, aircraft_state, 3, 3);
		System.arraycopy(velocity_body, 0, aircraft_state, 6, 3);
		System.arraycopy(omega_body, 0, aircraft_state, 9, 3);
	}

	/**
	 * First sets the ground state of the aircraft then begins the timed
	 * execution of the autopilot functionality.
	 * 
	 * @param view
	 *            The AUTOPILOT view. Allows the button to start the autopilot.
	 */
	public void startAutopilot(View view) {
		getGroundState();
		handler.postDelayed(runnable, 10);
	}

	/**
	 * updateECEF() computes the Earth Centered Earth Fixed (ECEF) position in
	 * XYZ from the WGS84 Ellipsoid.
	 * <p>
	 * Assigns: ECEF[] = Float containing the current XYZ position in ECEF
	 * coordinates.
	 */
	private void updateECEF() {
		double a = 6378137.0;
		double b = 6356752.314245;
		double ECC = Math.sqrt(1 - Math.pow(b, 2) / Math.pow(a, 2));
		double HEIGHT = GEOHEIGHT + SensorManager.getAltitude(p0, mPress[0]);
		double NPHI = a
				/ (Math.sqrt(1 - Math.pow(ECC, 2) * Math.pow(Math.sin(lat), 2)));

		ECEF[0] = (float) ((NPHI + HEIGHT) * Math.cos(lat * Math.PI / 180) * Math
				.cos(lon * Math.PI / 180));
		ECEF[1] = (float) ((NPHI + HEIGHT) * Math.cos(lat * Math.PI / 180) * Math
				.sin(lon * Math.PI / 180));
		ECEF[2] = (float) ((NPHI * (1 - Math.pow(ECC, 2)) + HEIGHT) * Math
				.sin(lat * Math.PI / 180));
	}

	/**
	 * Updates UI values to their current values from aircraft_state.
	 */
	public void updateValues() {
		XField.setText(String.valueOf(position_inertial[0]));
		YField.setText(String.valueOf(position_inertial[1]));
		ZField.setText(String.valueOf(position_inertial[2]));
		phiField.setText(String.valueOf(euler_angles[0] * 180 / Math.PI));
		thetaField.setText(String.valueOf(euler_angles[1] * 180 / Math.PI));
		psiField.setText(String.valueOf(euler_angles[2] * 180 / Math.PI));
		uField.setText(String.valueOf(velocity_body[0]));
		vField.setText(String.valueOf(velocity_body[1]));
		wField.setText(String.valueOf(velocity_body[2]));
		pField.setText(String.valueOf(omega_body[0]));
		qField.setText(String.valueOf(omega_body[1]));
		rField.setText(String.valueOf(omega_body[2]));
	}
}
