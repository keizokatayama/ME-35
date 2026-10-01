# 2D rotation matrix
from math import cos, sin, pi
import math
import numpy as np
import matplotlib.pyplot as plt
# remember that the default angles will be in radians
θ =  math.pi/4 # try changing this value
R = [
    [cos(θ), -sin(θ)],
    [sin(θ),  cos(θ)]
]
print(R)



x = 2
y = 3
p = [x, y]


theta_degrees = 90 #change this value
theta_radian = theta_degrees * pi / 180

p_rotated = [
    x*cos(theta_radian) - y*sin(theta_radian),
    x*sin(theta_radian) + y*cos(theta_radian)
]

print(p_rotated)



x = 2
y = 3
p = [x, y]

translation_x = 4
translation_y = 10

p_translated = [
    x + translation_x, y + translation_y
]

print(p_translated)


import matplotlib.pyplot as plt

# Define a point
point = np.array([2, 3, 1]) # this is a point at (2,3)


# we are translating this by (1,7)
translation_x = 1
translation_y = 7


# Create a homogeneous matrix for 45 degrees (pi/4 radians)
theta = np.pi/4
transformation_matrix = np.array([
    [np.cos(theta), -np.sin(theta), translation_x],
    [np.sin(theta), np.cos(theta), translation_y],
    [0,0,1]
])


# Apply the rotation
transformed_point = transformation_matrix @ point #@ is matrix multiplication

print(f"Original point: {point}")
print(f"Rotated point: {transformed_point}")

# Visualization
plt.figure(figsize=(5, 5))
plt.grid(True)
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)
plt.arrow(0, 0, point[0], point[1], head_width=0.1, head_length=0.1, fc='blue', ec='blue')
plt.arrow(0, 0, transformed_point[0], transformed_point[1], head_width=0.1, head_length=0.1, fc='red', ec='red')
plt.text(point[0], point[1], 'Original')
plt.text(transformed_point[0], transformed_point[1], 'Transformed')
plt.axis('equal')
plt.xlim(-20, 20)
plt.ylim(-20, 20)
plt.title('2D Transformation Example')
plt.show()


point = np.array([2, 3, 1]) # this is a point at (2,3)


# we are translating this by (2,0)
translation_x = 2
translation_y = 0
translation_matrix = np.array([
    [1,0,translation_x],
    [0,1,translation_y],
    [0,0,1]])


# Create a rotation matrix for 45 degrees (pi/4 radians)
theta = np.pi/4
rotation_matrix = np.array([
    [np.cos(theta), -np.sin(theta),0],
    [np.sin(theta), np.cos(theta),0],
    [0,0,1]
])


# First translation 
translated_point = translation_matrix @ point #@ is matrix multiplication

# Then rotation
rotated_point = rotation_matrix @ translated_point

print(f"Original point: {point}")
print(f"Translated point: {translated_point}")
print(f"Rotated point: {rotated_point}")


# Visualization
plt.figure(figsize=(5, 5))
plt.grid(True)
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)
plt.arrow(0, 0, point[0], point[1], head_width=0.1, head_length=0.1, fc='blue', ec='blue')
plt.arrow(0, 0, translated_point[0], translated_point[1], head_width=0.1, head_length=0.1, fc='red', ec='red')
plt.arrow(0, 0, rotated_point[0], rotated_point[1], head_width=0.1, head_length=0.1, fc='green', ec='green')
plt.text(point[0], point[1], 'Original')
plt.text(translated_point[0], translated_point[1], 'Translated')
plt.text(rotated_point[0], rotated_point[1], 'Rotated')

plt.axis('equal')
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.title('Translation then rotation')
plt.show()


point = np.array([2, 3, 1]) # this is a point at (2,3)


# we are translating this by (2,0)
translation_x = 2
translation_y = 0
translation_matrix = np.array([
    [1,0,translation_x],
    [0,1,translation_y],
    [0,0,1]])


# Create a rotation matrix for 45 degrees (pi/4 radians)
theta = np.pi/4
rotation_matrix = np.array([
    [np.cos(theta), -np.sin(theta),0],
    [np.sin(theta), np.cos(theta),0],
    [0,0,1]
])

# First rotation
rotated_point = rotation_matrix @ point

# then translation 
translated_point = translation_matrix @ rotated_point #@ is matrix multiplication


print(f"Original point: {point}")
print(f"Translated point: {translated_point}")
print(f"Rotated point: {rotated_point}")


# Visualization
plt.figure(figsize=(5, 5))
plt.grid(True)
plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)
plt.arrow(0, 0, point[0], point[1], head_width=0.1, head_length=0.1, fc='blue', ec='blue')
plt.arrow(0, 0, translated_point[0], translated_point[1], head_width=0.1, head_length=0.1, fc='red', ec='red')
plt.arrow(0, 0, rotated_point[0], rotated_point[1], head_width=0.1, head_length=0.1, fc='green', ec='green')
plt.text(point[0], point[1], 'Original')
plt.text(translated_point[0], translated_point[1], 'Translated')
plt.text(rotated_point[0], rotated_point[1], 'Rotated')

plt.axis('equal')
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.title('Translation then rotation')
plt.show()



