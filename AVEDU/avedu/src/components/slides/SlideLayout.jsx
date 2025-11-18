// =============================================================
// FILE: src/components/slides/SlideLayout.jsx
// Base reusable slide layout with navigation and progress
// =============================================================
import React from "react";
import "./SlideLayout.scss";

/**
 * Base layout for slides with consistent styling and navigation
 * @param {object} props
 * @param {React.ReactNode} props.children - Slide content
 * @param {string} props.title - Slide title
 * @param {string} [props.className] - Additional CSS classes
 * @param {object} [props.style] - Additional inline styles
 */
export function SlideLayout({ children, title, className = "", style = {} }) {
  return (
    <div className={`slide ${className}`} style={style}>
      {title && <h2>{title}</h2>}
      {children}
    </div>
  );
}

/**
 * Slide card/section component (glass morphism style)
 * @param {object} props
 * @param {React.ReactNode} props.children - Card content
 * @param {string} [props.title] - Card title
 * @param {string} [props.variant] - Layout variant: 'default', 'aside'
 * @param {string} [props.className] - Additional CSS classes
 */
export function SlideCard({ children, title, variant = "default", className = "" }) {
  const variantClass = variant === "aside" ? "slide-card--aside" : "";
  return (
    <div className={`slide-card ${variantClass} ${className}`}>
      {title && <div className="slide-card__title">{title}</div>}
      {children}
    </div>
  );
}

/**
 * Two-column layout for slides
 * @param {object} props
 * @param {React.ReactNode} props.left - Left column content
 * @param {React.ReactNode} props.right - Right column content
 * @param {string} [props.ratio] - Column ratio: '50-50', '30-70', '70-30'
 * @param {string} [props.className] - Additional CSS classes
 */
export function SlideColumns({ left, right, ratio = "50-50", className = "" }) {
  const ratioClass = ratio === "30-70" ? "slide-columns--30-70" : ratio === "70-30" ? "slide-columns--70-30" : "";
  return (
    <div className={`slide-columns ${ratioClass} ${className}`}>
      <div>{left}</div>
      <div>{right}</div>
    </div>
  );
}

/**
 * Code block with terminal styling
 * @param {object} props
 * @param {string} props.children - Code content
 * @param {string} [props.language] - Programming language (for future syntax highlighting)
 * @param {string} [props.className] - Additional CSS classes
 */
export function SlideCode({ children, language = "", className = "" }) {
  return (
    <pre className={`slide-code ${className}`} data-language={language}>
      {children}
    </pre>
  );
}

/**
 * Figure/Image container with optional caption
 * @param {object} props
 * @param {string} [props.src] - Image source URL
 * @param {string} [props.alt] - Image alt text
 * @param {string} [props.caption] - Figure caption
 * @param {React.ReactNode} [props.children] - Custom content (canvas, video, etc.)
 * @param {string} [props.className] - Additional CSS classes
 */
export function SlideFigure({ src, alt, caption, children, className = "" }) {
  return (
    <figure className={`slide-figure ${className}`}>
      {src ? <img src={src} alt={alt || ""} /> : children}
      {caption && <figcaption>{caption}</figcaption>}
    </figure>
  );
}

/**
 * Callout box with different variants
 * @param {object} props
 * @param {React.ReactNode} props.children - Callout content
 * @param {string} [props.variant] - Variant: 'info', 'warn', 'success', 'error'
 * @param {string} [props.className] - Additional CSS classes
 */
export function SlideCallout({ children, variant = "info", className = "" }) {
  const variantClass = `slide-callout--${variant}`;
  return <div className={`slide-callout ${variantClass} ${className}`}>{children}</div>;
}

/**
 * Action buttons container
 * @param {object} props
 * @param {React.ReactNode} props.children - Buttons
 * @param {string} [props.className] - Additional CSS classes
 */
export function SlideActions({ children, className = "" }) {
  return <div className={`slide-actions ${className}`}>{children}</div>;
}

/**
 * Button component for slides
 * @param {object} props
 * @param {React.ReactNode} props.children - Button text
 * @param {Function} [props.onClick] - Click handler
 * @param {boolean} [props.disabled] - Disabled state
 * @param {string} [props.variant] - Button variant: 'primary', 'secondary', 'danger'
 * @param {string} [props.className] - Additional CSS classes
 */
export function SlideButton({ children, onClick, disabled = false, variant = "primary", className = "" }) {
  return (
    <button className={`btn btn--${variant} ${className}`} onClick={onClick} disabled={disabled}>
      {children}
    </button>
  );
}

/**
 * Keyboard hint component
 * @param {object} props
 * @param {string} props.children - Key text (e.g., "Ctrl", "Space")
 */
export function SlideKbd({ children }) {
  return <kbd className="slide-kbd">{children}</kbd>;
}

/**
 * Progress indicator (dots)
 * @param {object} props
 * @param {number} props.total - Total number of slides
 * @param {number} props.current - Current slide index (0-based)
 * @param {Function} [props.onDotClick] - Handler when dot is clicked
 */
export function SlideProgress({ total, current, onDotClick }) {
  return (
    <div className="slide-progress">
      {Array.from({ length: total }, (_, i) => (
        <button
          key={i}
          className={`slide-progress__dot ${i === current ? "is-active" : ""}`}
          onClick={() => onDotClick?.(i)}
          aria-label={`Go to slide ${i + 1}`}
        />
      ))}
    </div>
  );
}

/**
 * Muted text utility
 */
export function SlideMuted({ children, className = "" }) {
  return <span className={`slide-muted ${className}`}>{children}</span>;
}

/**
 * Centered content utility
 */
export function SlideCenter({ children, className = "" }) {
  return <div className={`slide-center ${className}`}>{children}</div>;
}

// Default export for convenience
export default SlideLayout;
