---
name: ELIO Design System
colors:
  surface: '#f8faf9'
  surface-dim: '#d8dad9'
  surface-bright: '#f8faf9'
  surface-container-lowest: '#ffffff'
  surface-container-low: '#f2f4f3'
  surface-container: '#eceeed'
  surface-container-high: '#e6e9e8'
  surface-container-highest: '#e1e3e2'
  on-surface: '#191c1c'
  on-surface-variant: '#3d4943'
  inverse-surface: '#2e3131'
  inverse-on-surface: '#eff1f0'
  outline: '#6d7a73'
  outline-variant: '#bccac1'
  surface-tint: '#006c4e'
  primary: '#00694c'
  on-primary: '#ffffff'
  primary-container: '#008560'
  on-primary-container: '#f5fff7'
  inverse-primary: '#68dbae'
  secondary: '#506260'
  on-secondary: '#ffffff'
  secondary-container: '#d0e4e2'
  on-secondary-container: '#546665'
  tertiary: '#00647c'
  on-tertiary: '#ffffff'
  tertiary-container: '#007f9c'
  on-tertiary-container: '#fafdff'
  error: '#ba1a1a'
  on-error: '#ffffff'
  error-container: '#ffdad6'
  on-error-container: '#93000a'
  primary-fixed: '#86f8c9'
  primary-fixed-dim: '#68dbae'
  on-primary-fixed: '#002115'
  on-primary-fixed-variant: '#00513a'
  secondary-fixed: '#d3e6e4'
  secondary-fixed-dim: '#b7cac8'
  on-secondary-fixed: '#0d1e1d'
  on-secondary-fixed-variant: '#394a49'
  tertiary-fixed: '#b7eaff'
  tertiary-fixed-dim: '#4cd6ff'
  on-tertiary-fixed: '#001f28'
  on-tertiary-fixed-variant: '#004e60'
  background: '#f8faf9'
  on-background: '#191c1c'
  surface-variant: '#e1e3e2'
typography:
  h1:
    fontFamily: Sora
    fontSize: 40px
    fontWeight: '700'
    lineHeight: 48px
    letterSpacing: -0.02em
  h2:
    fontFamily: Sora
    fontSize: 32px
    fontWeight: '600'
    lineHeight: 40px
    letterSpacing: -0.01em
  h3:
    fontFamily: Sora
    fontSize: 24px
    fontWeight: '600'
    lineHeight: 32px
  body-lg:
    fontFamily: Inter
    fontSize: 18px
    fontWeight: '500'
    lineHeight: 28px
  body-md:
    fontFamily: Inter
    fontSize: 16px
    fontWeight: '500'
    lineHeight: 24px
  label-bold:
    fontFamily: Manrope
    fontSize: 14px
    fontWeight: '700'
    lineHeight: 20px
  label-sm:
    fontFamily: Manrope
    fontSize: 12px
    fontWeight: '600'
    lineHeight: 16px
rounded:
  sm: 0.25rem
  DEFAULT: 0.5rem
  md: 0.75rem
  lg: 1rem
  xl: 1.5rem
  full: 9999px
spacing:
  unit: 8px
  container-margin: 24px
  gutter: 16px
  stack-sm: 8px
  stack-md: 16px
  stack-lg: 32px
---

## Brand & Style

This design system embodies the "Eco-Tech" aesthetic, bridging the gap between cold industrial robotics and warm environmental stewardship. The brand personality is **Intelligent, Nurturing, and Precise**. It aims to evoke a sense of effortless sustainability, positioning the robot not just as a tool, but as a sophisticated partner in waste management.

The visual style is a fusion of **Minimalism** and **Glassmorphism**, specifically tailored for an IoT/AI context. It utilizes vast amounts of whitespace, high-clarity typography, and semi-transparent layers that suggest the transparency of AI processes. The aesthetic is polished and high-fidelity, using soft transitions and organic movement to make technical tasks feel approachable and professional in a light, clean environment.

## Colors

The palette is anchored by **Eco Green**, representing life and sustainability. This is paired with a **Deep Charcoal secondary** to provide the grounding "tech" contrast necessary for an AI product. A **Vibrant Cyan tertiary** is used sparingly for active "scanning" or "data-processing" states to reinforce the futuristic IoT theme.

As a **Light Mode** system, the interface utilizes clean neutral tones for the background to ensure high readability and a modern feel. Backgrounds should be layered using tonal shifts—moving from bright surfaces to subtle container tones—rather than heavy lines to maintain a seamless, integrated look.

## Typography

This design system utilizes a multi-font strategy to balance technical precision with functional clarity. **Sora** is used for headlines to provide a modern, industrial, yet approachable geometric feel. **Inter** is the primary typeface for body text, chosen for its exceptional readability in data-heavy AI interfaces. **Manrope** is reserved for labels and functional UI metadata, providing a highly legible and organized structure for technical details.

Weight is used as a primary hierarchy tool. Headlines should lean toward Bold (700) and SemiBold (600), while body text remains at Medium (500) to ensure readability. Letter spacing is slightly tightened on large headings to create a compact, modern appearance.

## Layout & Spacing

The layout follows a **Fluid Grid** model with an 8px base rhythm. Content is organized within a 12-column grid for desktop/tablet and a 4-column grid for mobile, emphasizing generous horizontal margins to create a "contained" and safe feel.

Spacing should favor "Stretching" over "Stacking." Use `stack-lg` to separate distinct functional modules and `stack-sm` for related metadata. Information density should be kept low to middle-range, allowing the minimalist aesthetic to flourish without cluttering the user's focus on the robot's status.

## Elevation & Depth

In Light Mode, hierarchy is established through **Tonal Layering** and **Glassmorphism**. Surfaces are distinguished by subtle value shifts from the background to container levels. When shadows are used, they are extra-diffused and low-opacity to create a subtle lift effect without appearing heavy.

To represent the "AI" aspect, use backdrop blurs (20px-30px) on top-level containers like navigation bars or floating action sheets. This creates a sense of depth and layering without using heavy borders. Surfaces should feel tactile and defined through the use of subtle 1px outlines on the edges of cards to catch virtual light.

## Shapes

The shape language is defined by the **8px base radius** (Rounded level 2), creating a soft, approachable, and "human-centric" geometry. This roundedness level avoids the sharpness of traditional industrial apps, moving instead toward a consumer-electronics feel.

Small elements like buttons and input fields use the 8px standard, while larger containers and cards may scale up to 16px or 24px (`rounded-xl`) to maintain visual harmony. Interactive components should never have sharp corners, reinforcing the friendly and safe nature of the ELIO robot.

## Components

### Buttons
Primary buttons are solid Eco Green with light text, utilizing a soft shadow for depth. Secondary buttons use a glass-like treatment: a semi-transparent light fill with a thin Eco Green border and dark text.

### Chips & Tags
Waste category chips (e.g., "Plastic," "Paper") use a subtle light-tint background with high-contrast text. They feature a 100px (pill) radius to distinguish them from actionable buttons.

### Cards
Cards are the primary container for data. In this light theme, they rely on tonal contrast and subtle 1px outlines rather than heavy shadows. Inner padding is strictly 24px to maintain the airy, minimalist feel.

### Input Fields
Inputs are rendered as light-grey troughs with a 1px border that glows Eco Green upon focus. Labels are always positioned above the field in `label-bold` style for maximum clarity.

### IoT Elements
Specialized components include a **"Robot Pulse" indicator** (a glowing green ring) to show the robot is active, and a **"Vision Viewfinder"** for the waste-sorting camera feed, which uses technical crosshairs and thin, high-tech stroke lines in Cyan.