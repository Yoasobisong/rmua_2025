#!/usr/bin/env python3

import pandas as pd
import plotly.graph_objects as go

# Read CSV file with only x and y coordinates
df = pd.read_csv('/data/workspace/rmua_2025/drone_ws/src/navigation_vision/position_fliter/only_windows_fliter.csv', 
                 header=None, usecols=[0, 1], names=['x', 'y'])

# Create figure with two subplots
fig = go.Figure()

# Add x position over time
fig.add_trace(
    go.Scatter(
        x=list(range(len(df))),  # Convert range to list
        y=df['x'],
        mode='lines+markers',
        name='X Position',
        line=dict(color='blue'),
        marker=dict(size=3)
    )
)

# Add y position over time
fig.add_trace(
    go.Scatter(
        x=list(range(len(df))),  # Convert range to list
        y=df['y'],
        mode='lines+markers',
        name='Y Position',
        line=dict(color='red'),
        marker=dict(size=3)
    )
)

# Update layout
fig.update_layout(
    title='X and Y Positions Over Time',
    xaxis_title='Time Index',
    yaxis_title='Position',
    width=1200,
    height=800,
    showlegend=True
)

# Save as PNG file
fig.write_image('/data/workspace/rmua_2025/drone_ws/src/navigation_vision/position_fliter/fliter.png')

print("Plot has been saved as 'only_windows_fliter.png'")
    