#!/usr/bin/env python3

import pandas as pd
import plotly.graph_objects as go
import numpy as np

# Read CSV file with only x and y coordinates (first two columns)
df = pd.read_csv('/data/workspace/rmua_2025/drone_ws/src/navigation_vision/position_fliter/position_data.csv', header=None, usecols=[0, 1], names=['x', 'y'])

# Create time index (0 to n-1)
df['time'] = np.arange(len(df))

# Create 3D scatter plot with lines
fig = go.Figure(data=[
    # Add lines connecting points
    go.Scatter3d(
        x=df['x'],
        y=df['y'],
        z=df['time'],
        mode='lines',
        line=dict(color='blue', width=2),
        name='Path'
    ),
    # Add points
    go.Scatter3d(
        x=df['x'],
        y=df['y'],
        z=df['time'],
        mode='markers',
        marker=dict(
            size=3,
            color='red',
            opacity=0.8
        ),
        name='Points'
    ),
    # Add center point line
    go.Scatter3d(
        x=[480, 480],
        y=[360, 360],
        z=[df['time'].min(), df['time'].max()],
        mode='lines',
        line=dict(color='red', width=2, dash='dash'),
        name='Image Center'
    )
])

# Update layout
fig.update_layout(
    title='Detection Points Path Over Time',
    scene=dict(
        xaxis_title='X Position',
        yaxis_title='Y Position',
        zaxis_title='Point Index',
        camera=dict(
            up=dict(x=0, y=0, z=1),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=1.5, y=1.5, z=1.5)
        )
    ),
    width=1200,
    height=800,
    showlegend=True
)

# Save as HTML file
fig.write_html('/data/workspace/rmua_2025/drone_ws/src/navigation_vision/position_fliter/position_visualization.html')

print("Interactive visualization has been saved as 'position_visualization.html'")
