package edu.cmu.ri.airboat.server;

import robotutils.Pose3D;
import robotutils.Quaternion;
import edu.cmu.ri.crw.VehicleFilter;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.Utm;
import edu.cmu.ri.crw.data.UtmPose;

/**
 * A basic filter that uses weighted averages and a first-order approximate
 * motion model to predict and update state.
 * 
 * @author pkv
 *
 */
public class SimpleFilter implements VehicleFilter {
	
	// The largest allowed numerical integration timestep
	// (larger intervals are integrated using multiple steps of this length)
	public static final long MAX_STEP_MS = 200;
	
	// Tuning factors for compass and GPS
	// 0.0 means no confidence, 1.0 means perfect accuracy.  Nominal is ~0.1.
	public static final double ALPHA_COMPASS = 0.1;
	public static final double ALPHA_GPS = 0.9;
	
	// Indicator variables used to mark whether absolute heading and position were measured
	boolean _isInitializedGps = false;
	boolean _isInitializedCompass = false;
	
	// State represented by 6D pose
	UtmPose _pose = new UtmPose(new Pose3D(476608.34, 4671214.40, 172.35, 0, 0, 0), new Utm(17, true));
	Twist _vels = new Twist();
	
	// The current time in milliseconds, used to measure filter update intervals
	long _time = System.currentTimeMillis();
	
	protected void predict(long time) {
		/*while(_time < time) {
			long step = Math.min(time - _time, MAX_STEP_MS);
			double dt = step / 1000.0;
			double yaw = _pose.pose.getRotation().toYaw();
			
			double x = _pose.pose.getX() + dt * (_vels.dx() * Math.sin(yaw) + _vels.dy() * Math.cos(yaw));
			double y = _pose.pose.getY() + dt * (_vels.dx() * Math.cos(yaw) - _vels.dy() * Math.sin(yaw));
			double z = _pose.pose.getZ();
			Quaternion rotation = Quaternion.fromEulerAngles(0, 0, _pose.pose.getRotation().toYaw() + dt * _vels.drz());
			_pose.pose = new Pose3D(x, y, z, rotation);
			
			_time += step;
		}
		*/
	}
	
	@Override
	public synchronized void compassUpdate(double yaw, long time) {
		// predict(time);
		
		_pose.pose = new Pose3D(_pose.pose.getX(), _pose.pose.getY(), _pose.pose.getZ(), 
				Quaternion.fromEulerAngles(0, 0, yaw));
		
		/*// On the first compass update, simply take on the initial heading
		if (_isInitializedCompass) {
			double oldYaw = _pose.pose.getRotation().toYaw();
			_pose.pose = new Pose3D(_pose.pose.getX(), _pose.pose.getY(), _pose.pose.getZ(), 
					Quaternion.fromEulerAngles(0, 0, angleAverage(ALPHA_COMPASS, oldYaw, yaw)));
		} else {
			_pose.pose = new Pose3D(_pose.pose.getX(), _pose.pose.getY(), _pose.pose.getZ(), 
					Quaternion.fromEulerAngles(0, 0, yaw));
			_isInitializedCompass = true;
		}
		*/
	}

	@Override
	public synchronized void gpsUpdate(UtmPose utm, long time) {
		predict(time);
		
		// If we are in the wrong zone or are uninitialized, use the GPS position
		if (!_pose.origin.equals(utm.origin) || !_isInitializedGps) {
			_pose.origin = utm.origin.clone();
			_pose.pose = utm.pose.clone();
			_isInitializedGps = true;
		} else {
			// On other update, average together the readings
			double x = ALPHA_GPS * utm.pose.getX() + (1 - ALPHA_GPS) * _pose.pose.getX();
			double y = ALPHA_GPS * utm.pose.getY() + (1 - ALPHA_GPS) * _pose.pose.getY();
			double z = (utm.pose.getZ() == 0.0) ? _pose.pose.getZ() : utm.pose.getZ();
			
			// If we have a bearing, use it as well (yaw != exactly 0 in valid quaternions)
			/*Quaternion orientation;
			if (utm.pose.getRotation().toYaw() != 0.0) {
				double oldYaw = _pose.pose.getRotation().toYaw();
				double yaw = utm.pose.getRotation().toYaw();
				orientation = Quaternion.fromEulerAngles(0, 0, angleAverage(ALPHA_GPS, oldYaw, yaw));
			} else {
				orientation = _pose.pose.getRotation();
			}
			*/
			Quaternion orientation = _pose.pose.getRotation();
			// Create new pose from update
			 _pose.pose = new Pose3D(x,y,z,orientation);
		}
	}

	@Override
	public synchronized void gyroUpdate(double yawVel, long time) {
		/*predict(time);
		_vels.drz(yawVel);
		*/
	}

	@Override
	public synchronized UtmPose pose(long time) {
		return _pose.clone();
	}

	@Override
	public synchronized void reset(UtmPose pose, long time) {
		_time = time;
		_pose = pose.clone();
		
		_isInitializedGps = true;
		_isInitializedCompass = true;
	}

	/**
	 * Helper function that reprojects any angle into the range of (-pi, pi]
	 * 
	 * @param angle the angle to be reprojected
	 * @return the reprojected angle, between -pi and pi
	 */
	protected double normalizeAngle(double angle) {
		while (angle > Math.PI)
			angle -= 2*Math.PI;
		while (angle <= -Math.PI)
			angle += 2*Math.PI;
		return angle;
	}
	
	/**
	 * Computes a weighted average of two angles, with correct ring math.  It takes a tuning 
	 * constant which weights between the two parameters. A weight of 0.0 will simply return
	 * the first angle, while a weight of 1.0 will simply return the second angle.  A weight
	 * of 0.5 corresponds to the arithmetic mean of the two angles.
	 * 
	 * @param weight tuning constant between the two angles. 
	 * @param angle1 the first angle
	 * @param angle2 the second angle
	 * @return the weighted average of the two angles
	 */
	protected double angleAverage(double weight, double angle1, double angle2) {
		
		// Find the difference between the two angles (should always be less than pi/2) 
		double diff = normalizeAngle(angle2 - angle1);
		
		// Use the weight to foreshorten this angular difference, then add to original angle
		return angle1 + (weight*diff);
	}
}
