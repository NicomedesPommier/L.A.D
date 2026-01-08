# RVizTFViewer Component

A reusable RViz viewer component specifically designed for TF (Transform) visualization with built-in header, status indicator, **frame visibility controls**, and configuration.

## Features

- **Frame Visibility Controls**: Toggle individual TF frames on/off to isolate specific transforms
- **Search Functionality**: Search for specific frames by name
- **Show/Hide All**: Quick toggle for all frames
- **Status Indicator**: Automatically shows connection status with color-coded badge
- **Collapsible Controls**: Frame controls panel can be collapsed to save space
- **Responsive**: Adapts to different screen sizes and layout modes
- **Reusable**: Can be used in any learning level or component

## Usage

```jsx
import RVizTFViewer from "../../components/visualization/RVizTFViewer";

function MyComponent() {
  const { ros, connected } = useRoslib();

  return (
    <RVizTFViewer
      ros={ros?.current}
      connected={connected}
      fixedFrame="world"
      showGrid={true}
      showAxes={true}
      axisLength={0.3}
      showFrameControls={true}
    />
  );
}
```

## Props

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `ros` | Object | - | ROS connection object |
| `connected` | boolean | `false` | ROS connection status (updates status indicator) |
| `fixedFrame` | string | `"world"` | Fixed frame for RViz visualization |
| `showGrid` | boolean | `true` | Show grid in RViz |
| `showAxes` | boolean | `true` | Show coordinate axes in RViz |
| `axisLength` | number | `0.3` | Length of the coordinate axes |
| `title` | string | `"TF Visualization (RViz)"` | Custom title for the viewer |
| `className` | string | `""` | Additional CSS classes |
| `showFrameControls` | boolean | `true` | Show frame visibility controls panel |
| `onFramesUpdate` | function | - | Callback when frames list updates: `(frames: string[]) => void` |

## Frame Visibility Controls

The component includes a collapsible panel that allows you to:

1. **Toggle Individual Frames**: Check/uncheck frames to show/hide them in the visualization
2. **Search Frames**: Type to filter frames by name (e.g., "left" shows "left_wheel", "left_sensor")
3. **Show/Hide All**: Click the 👁️ button to quickly toggle all frames
4. **View Count**: See how many frames are visible vs total (e.g., "3/7")

### Example Use Case

Perfect for learning scenarios:
- Show only the `left_wheel` frame to understand its position
- Hide all frames except `base_link` and `camera` to focus on sensor alignment
- Search for "wheel" to see all wheel-related frames

## Style Variants

The component supports several CSS variants for different use cases:

### Sidebar Mode
```jsx
<RVizTFViewer
  ros={ros?.current}
  connected={connected}
  className="rviz-tf-viewer--sidebar"
/>
```
Removes left border for seamless integration in sidebars.

### Borderless Mode
```jsx
<RVizTFViewer
  ros={ros?.current}
  connected={connected}
  className="rviz-tf-viewer--borderless"
/>
```
Removes all borders for use in grid/flex layouts.

### Compact Mode
```jsx
<RVizTFViewer
  ros={ros?.current}
  connected={connected}
  className="rviz-tf-viewer--compact"
/>
```
Smaller header and reduced padding for space-constrained layouts.

## Examples

### Basic TF Visualization
```jsx
<RVizTFViewer
  ros={ros?.current}
  connected={connected}
  fixedFrame="world"
/>
```

### Without Frame Controls
```jsx
<RVizTFViewer
  ros={ros?.current}
  connected={connected}
  fixedFrame="world"
  showFrameControls={false}
/>
```
Useful when you want a clean viewer without controls.

### With Frame Update Callback
```jsx
const [availableFrames, setAvailableFrames] = useState([]);

<RVizTFViewer
  ros={ros?.current}
  connected={connected}
  fixedFrame="world"
  onFramesUpdate={(frames) => {
    console.log("Detected frames:", frames);
    setAvailableFrames(frames);
  }}
/>
```

### Custom Configuration
```jsx
<RVizTFViewer
  ros={ros?.current}
  connected={connected}
  fixedFrame="base_link"
  showGrid={false}
  showAxes={true}
  axisLength={0.5}
  title="Robot TF Tree"
  showFrameControls={true}
/>
```

### In a Split Layout
```jsx
<div style={{ display: 'grid', gridTemplateColumns: '1fr 400px', gap: 0 }}>
  <div>Main content here</div>
  <RVizTFViewer
    ros={ros?.current}
    connected={connected}
    className="rviz-tf-viewer--sidebar"
  />
</div>
```

## Frame Visibility Workflow

```jsx
// Student learns about TF frames step by step:

// 1. All frames visible initially (default)
<RVizTFViewer ros={ros?.current} connected={connected} />

// 2. Student searches for "wheel" frames
//    - Auto-filters to show only matching frames in the list
//    - Visualization still shows all frames

// 3. Student unchecks "right_wheel"
//    - Only "left_wheel" remains visible in 3D view
//    - Student can focus on understanding this single frame

// 4. Student clicks "Hide All" then selects only "base_link" and "camera"
//    - Perfect for learning sensor-to-robot relationships
```

## Interactive Learning Scenarios

### Scenario 1: Understanding Wheel Frames
1. Student sees all robot frames
2. Searches for "wheel" → filters to wheel frames only
3. Hides right wheels → focuses on left side
4. Can see exactly how left_wheel relates to base_link

### Scenario 2: Sensor Alignment
1. Student hides all frames
2. Manually enables `base_link`, `camera`, `lidar`
3. Studies how sensors are positioned relative to robot base
4. Can toggle each sensor on/off to compare positions

### Scenario 3: Debug TF Tree
1. Many frames detected (10+)
2. Student searches for specific frame by name
3. Toggles visibility to isolate problematic transform
4. Can clearly see if frame is in wrong position

## Keyboard Shortcuts

When the search input is focused:
- **Escape**: Clear search
- **Enter**: Toggle first frame in filtered list

## Accessibility

- Checkboxes are keyboard navigable
- Clear visual feedback for hover/focus states
- Status indicators use both color and text
- Screen reader friendly labels

## File Locations

- Component: `src/components/visualization/RVizTFViewer.jsx`
- Styles: `src/styles/components/_rviz-tf-viewer.scss`
- Base Viewer: `src/components/visualization/RVizViewer.jsx`

## Technical Details

### How Frame Filtering Works

1. `RVizTFViewer` maintains a Set of visible frame names
2. Passes this as array to `RVizViewer` via `visibleFrames` prop
3. `RVizViewer` updates THREE.js object visibility based on this array
4. Changes are instant (no re-rendering of entire scene)

### Performance

- Frame visibility changes are instant (just toggles THREE.js object.visible)
- No scene recreation needed
- Efficient for any number of frames (tested with 50+ frames)
- Search uses simple string filtering (very fast)

## Related Components

- `RVizViewer`: Base RViz visualization component
- `LidarVisualizer`: Component for LiDAR point cloud visualization
