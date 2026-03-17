def generate_data():
    data = []
    for i in range(1, 101):  # 1 to 5 with 0.1 interval means 50 steps
        value = 1 + (i - 1) * 0.1
        # Format the value in scientific notation with 18 decimal places
        formatted_value = f"{value:.18e}"
        # Create a line with the formatted value repeated twice, followed by four zeros
        line = f"{formatted_value},0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00"
        data.append(line)
    return data

# Generate the data
data = generate_data()

# Print the data
for line in data:
    print(line)
# import numpy as np
# import matplotlib.pyplot as plt

# # =============================================
# # Parameters
# # =============================================
# straight_length = 5.0    # Length of straight segments (meters)
# turn_radius = 5.0        # Radius of the U-turn (meters)
# dl = 0.1                 # Step size for path generation (meters)

# # =============================================
# # Generate Segments
# # =============================================
# def generate_straight_segment(start_x, start_y, direction, length):
#     """Generate a straight line segment."""
#     steps = int(length / dl)
#     if direction == 'right':
#         x = start_x + np.linspace(0, length, steps)
#         y = start_y * np.ones(steps)
#     elif direction == 'left':
#         x = start_x - np.linspace(0, length, steps)
#         y = start_y * np.ones(steps)
#     elif direction == 'up':
#         y = start_y + np.linspace(0, length, steps)
#         x = start_x * np.ones(steps)
#     elif direction == 'down':
#         y = start_y - np.linspace(0, length, steps)
#         x = start_x * np.ones(steps)
#     return x, y

# def generate_rotated_semicircle(start_x, start_y, radius):
#     """Generate a semicircle rotated 90 degrees clockwise."""
#     # Angular range: from π/2 (up) to -π/2 (down) for clockwise rotation
#     theta = np.linspace(np.pi/2, -np.pi/2, int(np.pi * radius / dl))
    
#     # Parametric equations for rotated semicircle
#     x = start_x + radius * np.cos(theta)
#     y = start_y + radius * np.sin(theta)
#     return x, y-radius

# # =============================================
# # Build the U-Shaped Path
# # =============================================
# # Initial straight segment (forward)
# x1, y1 = generate_straight_segment(0, 0, 'right', straight_length)
# print(x1)
# print(y1)
# # Semicircular turn (180 degrees to the left)
# x2, y2 = generate_rotated_semicircle(x1[-1], y1[-1], turn_radius)

# # Final straight segment (backward)
# x3, y3 = generate_straight_segment(x2[-1], y2[-1], 'left', straight_length)

# # Combine all segments
# cx = np.concatenate((x1,x2,x3))
# cy = np.concatenate((y1,y2,y3))

# # =============================================
# # Save Waypoints to File
# # =============================================
# # with open('u_shape_waypoints.txt', 'w') as f:
# #     for i in range(len(cx)):
# #         # Format: x, y, yaw, curvature, speed, placeholder
# #         # (Yaw and curvature are placeholders here)
# #         line = f"{cx[i]:.18e},{cy[i]:.18e},0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00\n"
# #         f.write(line)
# with open('u_turn_waypoints.txt', 'w') as f:
#     for x, y in zip(cx, cy):
#         # Format: x, y, yaw, curvature, speed, placeholder
#         line = (
#             f"{x:.18e},"
#             f"{y:.18e},"
#             "0.000000000000000000e+00,"  # yaw placeholder
#             "0.000000000000000000e+00,"  # curvature placeholder
#             "0.000000000000000000e+00,"  # speed
#             "0.000000000000000000e+00\n" # placeholder
#         )
#         f.write(line)
# # =============================================
# # Visualize the Path
# # =============================================
# plt.figure(figsize=(10, 6))
# plt.plot(cx, cy, 'b-', linewidth=2, label='Path')
# plt.scatter(cx, cy, c='r', s=10, label='Waypoints', alpha=0.5)
# plt.title('U-Shaped Path with Explicit Segments')
# plt.xlabel('X (m)')
# plt.ylabel('Y (m)')
# plt.grid(True)
# plt.axis('equal')
# plt.legend()
# plt.show()