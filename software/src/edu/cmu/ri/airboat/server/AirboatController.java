package edu.cmu.ri.airboat.server;

import java.util.Timer;
import java.util.TimerTask;


import robotutils.Pose3D;
import edu.cmu.ri.crw.VehicleController;
import edu.cmu.ri.crw.VehicleServer;
import edu.cmu.ri.crw.data.Twist;
import edu.cmu.ri.crw.data.UtmPose;
import com.google.code.microlog4android.LoggerFactory;

/**
 * A library of available navigation controllers that are accessible through the
 * high-level API.
 * 
 * @author pkv
 * @author kss
 * 
 */
public enum AirboatController {

	
	/**
	 * This is the implementation of Yunde's controller. This controller also updates the velocities twist as done in POINT_AND_SHOOT,
	 * but instead of velocities, it passes in servo commands to the thruster and rudder. The process of converting desired velocities
	 * to thrust and angle commands used to be done in the Arduino but has been moved here. The thruster and rudder commands will be
	 * at indexes 0 and 5 respectively.
	 * UPDATE: renamed POINT_AND_SHOOT to be compatible with all code. 
	 */
	POINT_AND_SHOOT(new VehicleController() {
		// variable for monitoring previous destination angle for error calculation 
		private double prev_angle_destination = 0;
		private final com.google.code.microlog4android.Logger logger_osman = LoggerFactory
				.getLogger(); // logger to debug
		// variables for buffer and integration term
		final int BUFFER_SIZE = 100;
		double[] buffer = new double[BUFFER_SIZE];
		int bIndex = 0;
		double bSum = 0;
		
		@Override
		public void update(VehicleServer server, double dt) {
			Twist twist = new Twist();
			
			// Get the position of the vehicle
			UtmPose state = server.getPose();
			Pose3D pose = state.pose;
			
			// Get the current waypoint, or return if there are none
			UtmPose[] waypoints = server.getWaypoints();
			if (waypoints == null || waypoints.length <= 0) {
				server.setVelocity(twist);
				return;
			}
			Pose3D waypoint = waypoints[0].pose;
			
			double distanceSq = planarDistanceSq(pose, waypoint);
			
			if (distanceSq <= 25)
			{
				// if reached the target, reset the buffer and previous angle
				bIndex = 0;
				bSum = 0;
				buffer = new double[BUFFER_SIZE];
				prev_angle_destination = 0;
				
				// If we are "at" the destination, de-queue current waypoint
				UtmPose[] queuedWaypoints = new UtmPose[waypoints.length - 1];
				System.arraycopy(waypoints, 1, queuedWaypoints, 0,
						queuedWaypoints.length);
				server.startWaypoints(queuedWaypoints,
						AirboatController.POINT_AND_SHOOT.toString());
			}
			else
			{
				// ANGLE CONTROL SEGMENT
				
				// find destination angle between boat and waypoint position
				double angle_destination = angleBetween(pose, waypoint);
				
				// use compass information to get heading of the boat
				double angle_boat = pose.getRotation().toYaw();
				double angle_between = normalizeAngle(angle_destination - angle_boat);
				
				// use gyro information from arduino to get rotation rate of heading
				double[] _gyroReadings = ((AirboatImpl) server).getGyro();
				double drz = _gyroReadings[2];
				
				// use previous data to get rate of change of destination angle
				double angle_destination_change = (angle_destination - prev_angle_destination) / dt;
				double error = angle_between;
				bIndex++;
				if (bIndex == BUFFER_SIZE)
					bIndex = 0;
				bSum -= buffer[bIndex];
				bSum += error;
				buffer[bIndex] = error;
				
				// Define PID constants and boundary pos constants
				AirboatImpl server_impl = (AirboatImpl) server;
				double[] rudder_pids = server_impl.getGains(5);
				
				double pos = rudder_pids[0]*(angle_between) + rudder_pids[2]*(angle_destination_change - drz) + rudder_pids[1]*bSum;
				
				// Ensure values are within bounds
				if (pos < -1.0)
					pos = -1.0;
				else if (pos > 1.0)
					pos = 1.0;
				
				// THRUST CONTROL SEGMENT
				double[] thrust_pids = server_impl.getGains(0);
				double thrust = AirboatImpl.CONST_THRUST * thrust_pids[0];
				
				// update twist
				twist.dx(thrust);
				twist.drz(pos);
				// log relevant variables
				/*
				logger_osman.info("Waypoint: " + waypoint);
				logger_osman.info("DEBUG: " + distanceSq + " " + angle_destination + " " + angle_boat + " " + drz + " " +
						prev_angle_destination + " " + angle_destination_change + " " + pos + " " + thrust
						+ " " + bSum + " " + rudder_pids[0] + " " + rudder_pids[1] + " " + rudder_pids[2]);
				*/
				// update angle error
				prev_angle_destination = angle_destination;
				// Set the desired velocity
				server.setVelocity(twist);
				
			}
		}
	}),
	/**
	 * This controller simply cuts all power to the boat, letting it drift
	 * freely. It will not attempt to hold position or steer the boat in any
	 * way, and completely ignores the waypoint.
	 */
	STOP(new VehicleController() {
		@Override
		public void update(VehicleServer server, double dt) {
			server.setVelocity(new Twist());

		}
	}),
	PRIMITIVES(new VehicleController() {
		private final com.google.code.microlog4android.Logger logger = LoggerFactory
				.getLogger();
		private boolean log_boolean = true;
		@Override
		public void update(final VehicleServer server, double dt) {
			server.setVelocity(new Twist(AirboatImpl.DEFAULT_TWIST));
			
			final double[] first = new double[3]; // angle, time, thrust
			first[0] = server.getWaypoints()[0].pose.getX();
			first[1] = server.getWaypoints()[0].pose.getY() * 1000; // convert to ms
			first[2] = server.getWaypoints()[0].pose.getZ();
			
			final double[] second = new double[3];
			second[0] = server.getWaypoints()[1].pose.getX();
			second[1] = server.getWaypoints()[1].pose.getY() * 1000; // convert to ms
			second[2] = server.getWaypoints()[1].pose.getZ();
			
			final double[] third = new double[3]; // angle, time, thrust
			third[0] = server.getWaypoints()[2].pose.getX();
			third[1] = server.getWaypoints()[2].pose.getY() * 1000; // convert to ms
			third[2] = server.getWaypoints()[2].pose.getZ();
			
			final double[] fourth = new double[3];
			fourth[0] = server.getWaypoints()[3].pose.getX();
			fourth[1] = server.getWaypoints()[3].pose.getY() * 1000; // convert to ms
			fourth[2] = server.getWaypoints()[3].pose.getZ();
			
			if (first[1] < 0 || second[1] < 0 || third[1] < 0 || fourth[1] < 0)
			{
				logger.info("INVALID INPUT: DEFAULT ALL TO 5 SEC");
				first[1] = 5000; second[1] = 5000; third[1] = 5000; fourth[1] = 5000;
			}
			
			final Timer t = new Timer();
			final Timer log = new Timer();
			
			TimerTask first_step = new TimerTask() {
				@Override
				public void run() {
					// TODO Auto-generated method stub
					if (log_boolean) { logger.info("START"); log_boolean = false;}
					((AirboatImpl) server).setVelocity(new Twist(first[2], 0, 0, 0, 0, first[0]));
					// when go testing, use AirboatImpl.CONST_THRUST
				}
			};
			TimerTask second_step = new TimerTask() {
				@Override
				public void run() {
					// TODO Auto-generated method stub
					((AirboatImpl) server).setVelocity(new Twist(second[2], 0, 0, 0, 0, second[0]));
					
				}
			};
			TimerTask third_step = new TimerTask() {
				@Override
				public void run() {
					// TODO Auto-generated method stub
					((AirboatImpl) server).setVelocity(new Twist(third[2], 0, 0, 0, 0, third[0]));
					
				}
			};
			TimerTask fourth_step = new TimerTask() {
				@Override
				public void run() {
					// TODO Auto-generated method stub
					((AirboatImpl) server).setVelocity(new Twist(fourth[2], 0, 0, 0, 0, fourth[0]));
					
				}
			};
			TimerTask close = new TimerTask() {
				
				@Override
				public void run() {
					// TODO Auto-generated method stub
					if (!log_boolean) {logger.info("END"); log_boolean = true;}
					((AirboatImpl) server).setVelocity(new Twist(500, 0, 0, 0, 0, 90));
					((AirboatImpl) server).startWaypoints(new UtmPose[0], "POINT_AND_SHOOT");
					log.purge();
					log.cancel();
					t.purge();
					t.cancel();
				}
			};
			TimerTask logging = new TimerTask() {
				
				@Override
				public void run() {
					// TODO Auto-generated method stub
					// retrieve state information
					AirboatImpl server_impl = (AirboatImpl) server;
					double yawVel = server_impl.getGyro()[2];
					Pose3D pose = server_impl.getPose().pose;
					double xPos = pose.getX();
					double yPos = pose.getY();
					double heading = pose.getRotation().toYaw();
					double rudder = server_impl.getVelocity().drz();
					double thrust = server_impl.getVelocity().dx();
					
					logger.info("PROCESS: " + " " + rudder + " " + heading + " " + yawVel
							+ " " + xPos + " " + yPos + " " + thrust);
				}
			};
			
			t.schedule(first_step, 0);
			t.schedule(second_step, (long) first[1]);
			t.schedule(third_step, (long) (first[1] + second[1]));
			t.schedule(fourth_step, (long) (first[1] + second[1] + third[1]));
			t.schedule(close, (long) (first[1] + second[1] + third[1] + fourth[1]));
			log.scheduleAtFixedRate(logging, 0, 200); // 200 ms logging rate
		}
	}),
	
	/**
	 * This controller shoots a picture only if it moves to a significantly
	 * different pose
	 */
	SHOOT_ON_MOVE(new VehicleController() {
		UtmPose lastPose = new UtmPose();

		@Override
		public void update(VehicleServer server, double dt) {
			Twist twist = new Twist();

			// Get the position of the vehicle
			UtmPose state = server.getPose();
			Pose3D pose = state.pose;

			// Get the current waypoint, or return if there are none
			UtmPose[] waypoints = server.getWaypoints();
			if (waypoints == null || waypoints.length <= 0) {
				server.setVelocity(twist);
				return;
			}
			Pose3D waypoint = waypoints[0].pose;

			// TODO: handle different UTM zones!
			// Compute the distance and angle to the waypoint
			double distanceSq = planarDistanceSq(pose, waypoint);
			double angle = angleBetween(pose, waypoint)
					- pose.getRotation().toYaw();
			angle = normalizeAngle(angle);

			// Choose driving behavior depending on direction and where we are
			if (Math.abs(angle) > 1.0) {
				// If we are facing away, turn around first
				twist.dx(0.5);
				twist.drz(Math.max(Math.min(angle / 1.0, 1.0), -1.0));
			} else if (distanceSq >= 9.0) {
				// If we are far away, drive forward and turn
				twist.dx(Math.min(distanceSq / 10.0, 1.0));
				twist.drz(Math.max(Math.min(angle / 10.0, 1.0), -1.0));
			}

			// Set the desired velocity
			server.setVelocity(twist);

			System.out.println(isNovel(pose, waypoint, PlanningMethod.SIMPLE));
			// First check if we are actually set to capture
			if (lastPose == null
					|| isNovel(pose, waypoint, PlanningMethod.SIMPLE) > 0.8) {
				lastPose.pose = pose.clone();
				System.out.println("Should Capture now!!!!!");
				server.startCamera(1, 0.0, 640, 480);
			}

			// TODO: add next waypoint functionality back in!!
		}

		/**
		 * Takes a pose and determines whether the image to be taken is novel or
		 * not
		 * 
		 * @param pose
		 *            The current pose
		 * @param waypoint
		 *            The current waypoint
		 * @param method
		 *            The Planning Method to be implemented
		 * @return A weight of novelty between 0 and 1
		 */
		double isNovel(Pose3D pose, Pose3D waypoint, PlanningMethod method) {

			final double CAMERA_AOV = Math.PI / 180.0f * 30; // Assuming that
																// the angle of
																// view of the
																// camera is 30
																// Degrees
			final double OVERLAP_RATIO = 0.8f;
			final double EFFECTIVE_DISTANCE = 10.0; // The effective distance
													// till which the camera
													// resolution/detection is
													// trusted
			double novelty = 0.0;
			switch (method) {
			case SIMPLE:
				// To simply calculate if the new pose is different
				/*
				 * Capture if new pose is different as per a. Change in yaw b.
				 * Change in position
				 */

				// No need to worry about the waypoint, inconsequential
				double angle = Math.abs(pose.getRotation().toYaw()
						- lastPose.pose.getRotation().toYaw());
				double distance = Math
						.sqrt((lastPose.pose.getX() - pose.getX())
								* (lastPose.pose.getX() - pose.getX())
								+ (lastPose.pose.getY() - pose.getY())
								* (lastPose.pose.getY() - pose.getY()));

				// Assign half weight to yaw, and half to distance
				if (angle >= CAMERA_AOV * OVERLAP_RATIO) {
					// i.e. if the current yaw has changed more than the
					// previous orientation by greater than 30 degrees * overlap
					// factor

					// This is because ANY yaw greater than the angle of view
					// will have completely new info (Think sectors)
					novelty = 0.5 * angle / (CAMERA_AOV * OVERLAP_RATIO);

					// Assuming that the zone of overlap is not useful
					// information
				}

				novelty += (distance / EFFECTIVE_DISTANCE) * 0.5;

				break;
			case GEOMETRIC:
				throw new UnsupportedOperationException("Not implemented Yet!");

			}
			return novelty;
		}

	});

	public enum PlanningMethod {
		SIMPLE, GEOMETRIC
	};

	/**
	 * The controller implementation associated with this library name.
	 */
	public final VehicleController controller;

	/**
	 * Instantiates a library entry with the specified controller.
	 * 
	 * @param controller
	 *            the controller to be used by this entry.
	 */
	private AirboatController(VehicleController controller) {
		this.controller = controller;
	}

	/**
	 * Takes an angle and shifts it to be in the range -Pi to Pi.
	 * 
	 * @param angle
	 *            an angle in radians
	 * @return the same angle as given, normalized to the range -Pi to Pi.
	 */
	public static double normalizeAngle(double angle) {
		while (angle > Math.PI)
			angle -= 2 * Math.PI;
		while (angle < -Math.PI)
			angle += 2 * Math.PI;
		return angle;
	}

	/**
	 * Computes the squared XY-planar Euclidean distance between two points.
	 * Using the squared distance is cheaper (it avoid a sqrt), and for constant
	 * comparisons, it makes no difference (just square the constant).
	 * 
	 * @param a
	 *            the first pose
	 * @param b
	 *            the second pose
	 * @return the XY-planar Euclidean distance
	 */
	public static double planarDistanceSq(Pose3D a, Pose3D b) {
		double dx = a.getX() - b.getX();
		double dy = a.getY() - b.getY();
		return dx * dx + dy * dy;
	}

	/**
	 * Computes a direction vector from a source pose to a destination pose, as
	 * projected onto the XY-plane. Returns an angle representing the direction
	 * in the XY-plane to take if starting at the source pose to reach the
	 * destination pose.
	 * 
	 * @param a
	 *            the source (starting) pose
	 * @param b
	 *            the destination (final) pose
	 * @return an angle in the XY-plane (around +Z-axis) to get to destination
	 */
	public static double angleBetween(Pose3D src, Pose3D dest) {
		return Math.atan2((dest.getY() - src.getY()),
				(dest.getX() - src.getX()));
	}
}