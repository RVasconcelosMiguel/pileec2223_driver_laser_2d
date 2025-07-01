import matplotlib.pyplot as plt
import numpy as np

# Create initial data
x = np.random.rand(100)
y = np.random.rand(100)

# Create scatter plot
plt.scatter(x, y, c='blue', alpha=0.5)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Moving Point Cloud')

# Update the scatter plot dynamically
for i in range(100):
    # Generate new data for the point cloud
    x = np.random.rand(100)
    y = np.random.rand(100)
    
    # Clear the current scatter plot
    plt.cla()
    
    # Plot the updated scatter plot
    plt.scatter(x, y, c='blue', alpha=0.5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Moving Point Cloud')
    
    # Set plot limits if needed
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    
    # Pause for a short duration to create animation effect
    plt.pause(0.01)

# Keep the plot window open
plt.show()