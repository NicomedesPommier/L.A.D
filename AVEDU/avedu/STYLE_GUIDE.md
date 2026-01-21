# L.A.D Style Guide

This document defines the visual language for the L.A.D (Learning Autonomous Driving) platform, combining **neon retro gaming** aesthetics with **modern fluid design**.

---

## 🎨 Color System

### CSS Variables (Use These!)
**Never hardcode colors.** Always use CSS variables from `_variables.scss`.

```scss
// Neon Accents
--neon: #7df9ff;      // Cyan neon (primary accent)
--neon2: #ff5cf4;     // Magenta neon (secondary accent)

// Backgrounds
--bg-primary: #0b0f1a;    // Main background
--bg-secondary: #111c3a;  // Cards, sections
--bg-surface: rgba(0,0,0,0.35);  // Code blocks, inputs
--glass: rgba(255,255,255,0.06);  // Glass morphism panels

// Text
--text: #e6f1ff;      // Primary text
--text-dim: #a8b3d1;  // Secondary text
--text-muted: #9fb3c8; // Tertiary/disabled text

// Status
--success: #48bb78;
--error: #dc2626;
--warning: #f59e0b;
--info: #3b82f6;
```

### Semantic Color Classes
```html
<!-- Status dots/indicators -->
<span className="slide-dot slide-dot--success">●</span>  <!-- Green -->
<span className="slide-dot slide-dot--error">●</span>    <!-- Red -->
<span className="slide-dot slide-dot--warning">●</span>  <!-- Yellow -->
<span className="slide-dot slide-dot--info">●</span>     <!-- Blue -->
```

---

## 📐 Spacing System

Use consistent **0.75rem** based spacing grid:

| Size | Value | Usage |
|------|-------|-------|
| xs | 0.25rem | Tight gaps, chip padding |
| sm | 0.5rem | Button gaps, small margins |
| md | 0.75rem | Standard gaps, card padding |
| lg | 1rem | Section breaks |
| xl | 1.5rem | Major separations |
| xxl | 2rem | Hero sections only |

**Classes:**
```html
<div className="slide-gap-sm">...</div>  <!-- gap: 0.5rem -->
<div className="slide-gap-md">...</div>  <!-- gap: 0.75rem -->
<div className="slide-gap-lg">...</div>  <!-- gap: 1rem -->
```

---

## 🧩 Component Patterns

### Cards
```jsx
// Standard card
<div className="slide-card">
  <div className="slide-card__title">Title</div>
  <p>Content</p>
</div>

// Two-column aside layout
<div className="slide-card slide-card--aside">
  <div>Left content</div>
  <div>Right sidebar</div>
</div>
```

### Callouts
```jsx
<div className="slide-callout slide-callout--info">
  <b>Key Concept:</b> Important information here.
</div>

// Types: --info, --warn, --success, --error
```

### Grids
```jsx
// 2-column equal
<div className="slide-grid slide-grid--2">...</div>

// 4-column equal
<div className="slide-grid slide-grid--4">...</div>

// 30/70 split
<div className="slide-grid slide-grid--30-70">...</div>
```

### Featured Boxes (Pipeline Diagrams)
```jsx
// Neon cyan accent
<div className="slide-featured">
  <b>Step 1</b>
  <p>Description</p>
</div>

// Magenta variant
<div className="slide-featured slide-featured--magenta">...</div>

// Orange, yellow, green variants
<div className="slide-featured slide-featured--orange">...</div>
<div className="slide-featured slide-featured--yellow">...</div>
<div className="slide-featured slide-featured--green">...</div>
```

### Pipeline/Flow Arrows
```jsx
<div className="slide-pipeline">
  <div className="slide-featured">Step 1</div>
  <div className="slide-pipeline__arrow">→</div>
  <div className="slide-featured">Step 2</div>
</div>
```

---

## 🎛️ Interactive Controls

### Sliders
```jsx
<div className="slide-slider">
  <span className="slide-slider__label">
    Speed: <span className="slide-slider__value">{value} m/s</span>
  </span>
  <input
    type="range"
    className="slide-slider__input"
    min="0" max="10" step="0.1"
    value={value}
    onChange={...}
  />
</div>
```

### Control Panels
```jsx
<div className="slide-controls">
  <button className="btn btn--primary">Action</button>
  <button className="btn">Secondary</button>
</div>
```

### Buttons
```jsx
<button className="btn">Default Button</button>
<button className="btn btn--primary">Primary Action</button>
<button className="btn btn--sm">Small Button</button>
```

---

## 📝 Typography

### Headings
```jsx
<h2>Slide Title</h2>  // Uses --neon color, neon glow effect
```

### Code Blocks
```jsx
<div className="slide-code">
  R = L / tan(δ)
</div>

// Or using pre tag
<pre className="slide-code">{codeString}</pre>
```

### Size Modifiers
```html
<span className="slide-text--sm">Smaller text (0.85rem)</span>
<span className="slide-text--lg">Larger text (1.1rem)</span>
```

---

## ❌ DON'T Do This

```jsx
// ❌ BAD: Inline styles
<div style={{ display: "grid", gap: "1rem", padding: "1.5rem" }}>

// ❌ BAD: Hardcoded colors
<div style={{ background: "#7df9ff", color: "#ff5cf4" }}>

// ❌ BAD: Mixing inline with classes
<div className="slide-card" style={{ marginTop: "1rem" }}>
```

## ✅ DO This

```jsx
// ✅ GOOD: Use classes
<div className="slide-grid slide-grid--2 slide-gap-lg">

// ✅ GOOD: Use CSS variables (for dynamic needs only)
<div style={{ color: "var(--neon)" }}>

// ✅ GOOD: Pure class-based styling
<div className="slide-card slide-mt-md">
```

---

## 🔧 When Inline Styles Are OK

Only use inline `style={{}}` for:
1. **Dynamic computed values** (canvas width, transform positions)
2. **Animation states** (e.g., progress bars)
3. **Conditional colors from data** (e.g., sensor status)

Always use CSS variables even in inline styles:
```jsx
style={{ borderColor: isActive ? "var(--neon)" : "var(--border)" }}
```
