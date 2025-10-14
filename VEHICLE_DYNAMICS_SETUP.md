# Vehicle Dynamics Module - Setup Guide

## Overview

A complete interactive Vehicle Dynamics learning module has been created for the L.A.D platform, covering fundamental concepts up to Chapter 4 of vehicle dynamics theory. The module includes 6 interactive slides with visualizations and real-time parameter controls.

## Files Created

### 1. Main Level Component
- **Location:** `AVEDU/avedu/src/levels/VehicleDynamics.jsx`
- **Description:** Main component that handles slide navigation and progression
- **Features:**
  - Auto-imports all slides from slidesVehicleDynamics directory
  - Keyboard navigation (← → arrows)
  - Progress tracking with visual dots
  - Mark completed functionality
  - Level completion button on final slide

### 2. Slide Components

#### Slide 01: Introduction (`slidesVehicleDynamics/01-Introduction.jsx`)
- Welcome slide explaining vehicle dynamics
- Key topics overview
- Applications in autonomous vehicles and control systems
- Non-interactive (informational)

#### Slide 02: Instantaneous Center of Rotation (`slidesVehicleDynamics/02-InstantaneousCenterRotation.jsx`)
- **Interactive Features:**
  - Real-time canvas visualization of vehicle from top view
  - Adjustable steering angle (-30° to 30°)
  - Adjustable wheelbase (2.0m to 4.0m)
  - Adjustable track width (1.2m to 2.0m)
- **Visualizations:**
  - Vehicle body with front/rear wheels
  - ICR point calculation and display
  - Turning radius visualization
  - Dynamic turning circle arc
- **Formula:** R = L / tan(δ)

#### Slide 03: Ackermann Steering Geometry (`slidesVehicleDynamics/03-AckermannSteering.jsx`)
- **Interactive Features:**
  - Real-time visualization of Ackermann geometry
  - Inner wheel angle control (-30° to 30°)
  - Automatic outer wheel angle calculation
  - Vehicle parameter adjustment (wheelbase, track width)
- **Visualizations:**
  - Color-coded inner (pink) and outer (cyan) wheels
  - Lines from wheels to ICR point
  - Angle difference display
- **Formula:** cot(δₒ) = cot(δᵢ) + t/L

#### Slide 04: Bicycle Model (`slidesVehicleDynamics/04-BicycleModel.jsx`)
- **Interactive Features:**
  - Adjustable lf (0.8m to 2.0m) - CG to front axle
  - Adjustable lr (0.8m to 2.0m) - CG to rear axle
  - Steering angle control (-30° to 30°)
  - Sideslip angle control (-15° to 15°)
  - Velocity adjustment (0 to 30 m/s)
  - Toggle velocity vectors on/off
- **Visualizations:**
  - 2-wheel simplified vehicle model
  - Center of gravity (CG) marker
  - Velocity vectors at CG and front wheel
  - Coordinate system display
- **Equations:**
  - dx/dt = V·cos(ψ + β)
  - dy/dt = V·sin(ψ + β)
  - dψ/dt = (V/L)·tan(δ)

#### Slide 05: Tire Forces and Slip Angles (`slidesVehicleDynamics/05-TireForcesSlipAngles.jsx`)
- **Interactive Features:**
  - Slip angle control (-15° to 15°)
  - Cornering stiffness adjustment (40,000 to 120,000 N/rad)
  - Normal force adjustment (2,000 to 8,000 N)
- **Visualizations:**
  - Tire top view with tread pattern
  - Velocity direction vs wheel orientation
  - Slip angle arc visualization
  - Lateral force vector display
  - Tire characteristic curve (force vs slip angle)
  - Linear region indicator on curve
- **Formula:** Fy = -Cα · α

#### Slide 06: Lateral Dynamics & Stability (`slidesVehicleDynamics/06-LateralDynamics.jsx`)
- **Interactive Features:**
  - Mass adjustment (1000 to 2500 kg)
  - lf adjustment (0.8m to 2.0m)
  - lr adjustment (0.8m to 2.0m)
  - Front cornering stiffness Cf (40,000 to 120,000 N/rad)
  - Rear cornering stiffness Cr (40,000 to 120,000 N/rad)
  - Velocity adjustment (5 to 40 m/s)
- **Real-time Calculations:**
  - Understeer gradient K
  - Stability classification (Understeer/Neutral/Oversteer)
  - Critical speed calculation
  - Yaw moment of inertia
- **Visualizations:**
  - Color-coded stability indicator
  - Understeer (green), Neutral (cyan), Oversteer (pink)
  - Critical speed warning
- **Equations:**
  - Lateral: m(dvy/dt + vx·r) = Fyf + Fyr
  - Yaw: Iz·dr/dt = lf·Fyf - lr·Fyr
  - Understeer gradient: K = (m/L²)·(lf/Cf - lr/Cr)

### 3. Routing Integration
- **File Modified:** `AVEDU/avedu/src/pages/LearnLevel.jsx`
- **Changes:**
  - Imported VehicleDynamics component
  - Added "vehicle-dynamics" to REGISTRY
  - Level slug: `vehicle-dynamics`

## Backend Setup (Required)

### Database Configuration

You need to add the Vehicle Dynamics level to your Django database. Here are the steps:

#### Option 1: Django Admin Interface

1. Start your Django server:
   ```bash
   cd LAD/lad
   python manage.py runserver
   ```

2. Go to `http://localhost:8000/admin/`

3. **Create/Select a Unit:**
   - Navigate to Learning → Units
   - Create a new unit or select an existing one (e.g., "Control & Dynamics")
   - Note the slug (e.g., `control-dynamics`)

4. **Create the Level:**
   - Navigate to Learning → Levels
   - Click "Add Level"
   - Fill in:
     - **Slug:** `vehicle-dynamics`
     - **Unit:** Select the unit you created/chose
     - **Title:** `Vehicle Dynamics`
     - **Order:** Set appropriate order (e.g., 10)
     - **Is active:** ✓ (checked)
   - Save

5. **Create Objectives:**
   - Navigate to Learning → Objectives
   - Create objectives for each slide:

   | Code | Description | Points | Level |
   |------|-------------|--------|-------|
   | `vd-slide-intro` | Complete Introduction to Vehicle Dynamics | 10 | vehicle-dynamics |
   | `vd-slide-icr` | Understand Instantaneous Center of Rotation | 15 | vehicle-dynamics |
   | `vd-slide-ackermann` | Master Ackermann Steering Geometry | 15 | vehicle-dynamics |
   | `vd-slide-bicycle` | Understand the Bicycle Model | 20 | vehicle-dynamics |
   | `vd-slide-tire-forces` | Learn Tire Forces and Slip Angles | 20 | vehicle-dynamics |
   | `vd-slide-lateral` | Master Lateral Dynamics & Stability | 20 | vehicle-dynamics |

#### Option 2: Django Shell

Run this Python script in Django shell:

```bash
cd LAD/lad
python manage.py shell
```

```python
from apps.learning.models import Unit, Level, Objective

# Create or get unit
unit, created = Unit.objects.get_or_create(
    slug="control-dynamics",
    defaults={
        "title": "Control & Dynamics",
        "order": 10,
        "is_active": True
    }
)

# Create level
level, created = Level.objects.get_or_create(
    slug="vehicle-dynamics",
    defaults={
        "unit": unit,
        "title": "Vehicle Dynamics",
        "order": 10,
        "is_active": True
    }
)

# Create objectives
objectives_data = [
    ("vd-slide-intro", "Complete Introduction to Vehicle Dynamics", 10),
    ("vd-slide-icr", "Understand Instantaneous Center of Rotation", 15),
    ("vd-slide-ackermann", "Master Ackermann Steering Geometry", 15),
    ("vd-slide-bicycle", "Understand the Bicycle Model", 20),
    ("vd-slide-tire-forces", "Learn Tire Forces and Slip Angles", 20),
    ("vd-slide-lateral", "Master Lateral Dynamics & Stability", 20),
]

for code, desc, points in objectives_data:
    Objective.objects.get_or_create(
        code=code,
        defaults={
            "level": level,
            "description": desc,
            "points": points
        }
    )

print("✓ Vehicle Dynamics level created successfully!")
```

## Testing

### 1. Start the Frontend
```bash
cd AVEDU/avedu
npm start
```

### 2. Start the Backend
```bash
cd LAD/lad
python manage.py runserver
```

### 3. Access the Level
Navigate to: `http://localhost:3000/learn/control-dynamics/vehicle-dynamics`
(Replace `control-dynamics` with your actual unit slug)

## Features Summary

### Interactive Elements
- **6 slides** with progressive complexity
- **14 adjustable parameters** across all slides
- **7 real-time canvas visualizations**
- **Real-time calculations** and formula displays
- **Color-coded visual feedback**

### User Experience
- Keyboard navigation (← →)
- Visual progress indicators (dots)
- Smooth transitions
- Responsive design
- Neon + Glass styling (matches app theme)

### Educational Content
- Covers fundamentals through Chapter 4
- Clear formulas and equations
- Visual representations of abstract concepts
- Interactive parameter exploration
- Real-world applications and design considerations

## Style Consistency

The module uses the existing L.A.D platform styling:
- `.slide` and `.slide-wrap` containers
- `.slide-card` for content sections
- `.slide-code` for formulas and code blocks
- `.slide-callout` variants (info, warn, success, error)
- `.slide-figure` for visualizations
- Neon cyan (#7df9ff) and pink (#ff5cf4) accent colors
- Glass morphism effects
- Consistent with RosBasic and Turtlesim levels

## Future Enhancements (Optional)

### Potential Additions:
1. **Slide 07:** Longitudinal Dynamics (acceleration, braking)
2. **Slide 08:** Combined Slip (friction circle)
3. **Slide 09:** Vehicle Stability Control (ESP/ESC)
4. **Slide 10:** Path Following and Trajectory Tracking
5. Interactive simulation where users can "drive" a vehicle
6. Quiz/assessment slide
7. Export parameters to JSON for use in simulations
8. 3D visualizations using Three.js or similar

## Troubleshooting

### Issue: "Level not found"
- **Solution:** Make sure the level slug in the database matches exactly: `vehicle-dynamics`

### Issue: "No slides found"
- **Solution:** Check that all slide files are in `AVEDU/avedu/src/levels/slidesVehicleDynamics/`

### Issue: Canvas not rendering
- **Solution:** Check browser console for errors. Ensure React 19 and canvas refs are working properly.

### Issue: Objectives not tracking
- **Solution:** Verify objective codes in database match the `meta.objectiveCode` in each slide file

## Summary

You now have a complete, interactive Vehicle Dynamics module that:
- ✓ Follows the app's existing architecture and style
- ✓ Covers fundamental concepts up to Chapter 4
- ✓ Provides hands-on learning through interactive visualizations
- ✓ Integrates seamlessly with the progress tracking system
- ✓ Maintains consistency with existing ROS and Turtlesim levels

Access it at: `/learn/{unit-slug}/vehicle-dynamics`

Enjoy teaching vehicle dynamics! 🚗💨
