import pyrealsense2 as rs
import cv2
import numpy as np
from collections import defaultdict, deque
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter


def draw_axis(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.05):
    # Define the 3D axis points
    axis = np.float32([[length, 0, 0], [0, length, 0], [0, 0, length], [0, 0, 0]]).reshape(-1, 3)
    # Project 3D points to the image plane
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = np.int32(imgpts).reshape(-1, 2)

    # Draw the axes
    cv2.line(img, tuple(imgpts[3]), tuple(imgpts[0]), (0, 0, 255), 3)  # X-axis in red
    cv2.line(img, tuple(imgpts[3]), tuple(imgpts[1]), (0, 255, 0), 3)  # Y-axis in green
    cv2.line(img, tuple(imgpts[3]), tuple(imgpts[2]), (255, 0, 0), 3)  # Z-axis in blue

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Load ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
aruco_params = cv2.aruco.DetectorParameters()

# Optionally, customize some of the parameters:
aruco_params.adaptiveThreshConstant = 10  # Threshold for adaptive thresholding
aruco_params.minMarkerPerimeterRate = 0.03  # Minimum perimeter ratio
aruco_params.maxMarkerPerimeterRate = 4.0  # Maximum perimeter ratio
aruco_params.perspectiveRemovePixelPerCell = 5  # Helps with perspective distortion
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Refinement for corner detection


# Camera calibration parameters (replace these with your camera's parameters)
# camera_matrix = np.array([[770.16035285, 0., 626.77686216],
#                           [0., 784.89518486, 310.00077159],
#                           [0., 0., 1.]])
# fx = 1050.  # Focal length in pixels (x-axis)
# fy = 700.  # Focal length in pixels (y-axis)
# cx = 1920 / 2.  # Principal point x-coordinate (center of the image)
# cy = 1080 / 2.  # Principal point y-coordinate (center of the image)


dist_coeffs = np.array([0.0439427, -0.01186024, 0.00350474, -0.00277281, -0.07163242])
# dist_coeffs = np.array([0, 0, 0, 0, 0])
# Marker side length in meters
marker_length = 0.20

# Store recent positions for each marker ID
recent_positions = defaultdict(lambda: deque(maxlen=5))

# Variables to store the saved position and orientation of ID 4
saved_tvec = None
saved_rvec = None

pos_diff = [0,0,0]
rvec_diff = [0,0,0]
id4_trajectory_xy = []
id4_trajectory_pixels = []

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()

    # Camera Intrinsic Parameters
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy
        camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]])
        # Convert frame to array and grayscale
        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray_image)

        id4_found = False
        current_id4_avg_pos = None
        current_id4_rvec = None

        if markerIds is not None:
            cv2.aruco.drawDetectedMarkers(color_image, markerCorners, markerIds)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, marker_length, camera_matrix, dist_coeffs)

            for i, marker_id in enumerate(markerIds):

                # Add current position to queue
                position = tvecs[i].ravel()
                recent_positions[marker_id[0]].append(position)
                avg_position = np.mean(recent_positions[marker_id[0]], axis=0)

                # Display averaged position
                position_text = f"ID {marker_id[0]} Avg: ({avg_position[0]+2.0:.3f}, {avg_position[1]+1.0:.3f}, {avg_position[2]:.3f})"
                corner = markerCorners[i][0][0]
                print(f"Marker ID: {marker_id[0]} - Avg Translation: {avg_position}")

                # Check if this is marker ID 4
                if marker_id[0] == 4:
                    id4_found = True
                    current_id4_avg_pos = avg_position
                    current_id4_rvec = rvecs[i].ravel()
                    
                    x_translation = -0.185  # Offset along the marker's Y-axis (in meters)

                    # Convert rvecs[i] to a rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])  # rvecs[i] -> 3x3 rotation matrix

                    # Translation in the Aruco marker's frame
                    translation_marker_frame = np.array([x_translation, 0, 0])  # [dx, dy, dz] in marker frame

                    # Transform the translation to the camera frame
                    translation_camera_frame = rotation_matrix @ translation_marker_frame  # Matrix multiplication

                    # Add the transformed translation to tvecs[i]
                    tvecs[i] += translation_camera_frame
                    
                    x_pixel = int((tvecs[i][0][0] * fx) / tvecs[i][0][2] + cx)
                    y_pixel = int((tvecs[i][0][1] * fy) / tvecs[i][0][2] + cy)
                    
                    # Add the transformed pixel coordinates to a list
                    id4_trajectory_pixels.append((x_pixel, y_pixel))

                    # Draw the trajectory onto the color frame
                    for j in range(1, len(id4_trajectory_pixels)):
                        if id4_trajectory_pixels[j - 1] is not None and id4_trajectory_pixels[j] is not None:
                            cv2.line(color_image, id4_trajectory_pixels[j - 1], id4_trajectory_pixels[j], (0, 255, 0), 2)

                    # Optionally, draw a circle at the current position
                    cv2.circle(color_image, (x_pixel, y_pixel), 5, (0, 0, 255), -1)
                    
                    id4_trajectory_xy.append(tvecs[i].ravel()[:2]) 
                    # tvecs[i][0][1] -= 0.2
                    # position_text2 = f"ID {marker_id[0]} Avg: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
                    cv2.putText(color_image, position_text, (int(corner[0] - 80), int(corner[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                    draw_axis(color_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length / 2)
                else:
                    cv2.putText(color_image, position_text, (int(corner[0] - 80), int(corner[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                    draw_axis(color_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length / 2)

        # Check for user input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Save position and orientation of ID 4 if found
            if id4_found:
                saved_tvec = current_id4_avg_pos.copy()
                saved_rvec = current_id4_rvec.copy()
                print("Saved ID 4 position and orientation.")
        elif key == ord('c'):
            # Calculate difference if ID 4 is found and we have saved data
            if id4_found and saved_tvec is not None and saved_rvec is not None:
                pos_diff = current_id4_avg_pos - saved_tvec
                rvec_diff = current_id4_rvec - saved_rvec
                print("Difference in position:", pos_diff)
                print("Difference in orientation (rvec):", rvec_diff)
                
        distance_diff_text = f"Position XYZ : ({pos_diff[0]:.3f}, {pos_diff[1]:.3f}, {pos_diff[2]:.3f})"
        orientation_diff_text = f"Orientation XYZ : ({rvec_diff[0]:.3f}, {rvec_diff[1]:.3f}, {rvec_diff[2]:.3f})"        
        cv2.putText(color_image, distance_diff_text, (0, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        cv2.putText(color_image, orientation_diff_text, (0, 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        # Display the output
        
        cv2.imshow("Detection", color_image)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

    if id4_trajectory_xy:
        id4_trajectory_xy = np.array(id4_trajectory_xy)
        plt.figure()
        plt.plot(id4_trajectory_xy[:, 0], id4_trajectory_xy[:, 1], label='ID 4 Trajectory')
        plt.scatter(id4_trajectory_xy[:, 0], id4_trajectory_xy[:, 1], c='r', marker='o')  # Optional: points
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title("Trajectory of Marker ID 4 (X-Y)")
        plt.legend()
        plt.grid()
            # Format the axes ticks to .00
        plt.gca().xaxis.set_major_formatter(FormatStrFormatter('%.3f'))
        plt.gca().yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
        plt.show()