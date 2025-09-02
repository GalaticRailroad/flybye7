# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Unity 6 project using DOTS (Data-Oriented Technology Stack) that implements a large-scale space simulation game called "Galaxy Sample". The game simulates teams of spaceships fighting for control of planets using Unity's Entity Component System (ECS).

## Essential Commands

**Running the Game:**
- Open the `Main` scene in Unity Editor and press Play
- Use Escape to toggle the in-game menu for settings
- Camera controls: WASD + Mouse (when menu hidden), Z for camera modes, X to switch targets

**Build Profiles:**
- Windows Dev build: `Assets/Settings/Build Profiles/Windows_Dev.asset`
- Windows Release build: `Assets/Settings/Build Profiles/Windows_Release.asset`

**Unity Version:**
- Check `ProjectSettings/ProjectVersion.txt` for minimum supported Unity version
- Currently configured for Unity 6 with URP (Universal Render Pipeline)

## Core Architecture

### ECS System Organization

The project follows Unity DOTS architecture with these key system groups:
- `BeginSimulationMainThreadGroup` - Game initialization and setup
- `BuildSpatialDatabaseGroup` - Spatial partitioning for performance
- Main simulation systems run after spatial database building

### Key Systems (Update Order Matters)

1. **GameInitializeSystem** - Handles all game initialization when `Config.MustInitializeGame` is true
2. **TeamAISystem** - Computes AI actions and importance scores for all entities
3. **ShipSystem** - Core ship logic (navigation, AI decisions, combat)
4. **BuildingSystem** - Factory production, turret combat, research bonuses
5. **PlanetSystem** - Resource generation, conversion, ship assessment
6. **VFXSystem** - High-performance VFX using graphics buffers instead of GameObjects

### Component Architecture

**Ship Types:** All extend base `Ship` component
- `Fighter` - Combat ships with attack capabilities
- `Worker` - Capture planets and construct buildings
- `Trader` - Resource distribution between planets

**Core Components:**
- `Initialize` (IEnableableComponent) - Marks entities needing initialization
- `Health` - HP system for ships and buildings
- `Team` - Team assignment for AI and combat
- `ExecuteAttack/ExecutePlanetCapture/ExecuteBuild/ExecuteTrade` - Action execution markers

### Blob Asset Data Pattern

The project uses blob assets for shared, immutable data:
- `ShipData`, `FighterData`, `WorkerData`, `TraderData` for ship types
- `BuildingData`, `TurretData` for buildings
- Implemented via `IBlobAuthoring<T>` interface in `BlobAuthoring` utility classes
- Reduces memory footprint and improves chunk layout performance

### AI System (Utility AI)

The AI uses a utility-based system in `TeamAISystem`:
- Computes statistics about empire state (planets, ships, resources)
- Generates action lists with importance scores:
  - `FighterAction` - Which planets to attack/defend
  - `WorkerAction` - Which planets to capture or buildings to construct
  - `TraderAction` - Resource distribution routes
  - `FactoryAction` - Which ship types to produce
- Individual entities choose actions via weighted random selection with personal bias

### Performance Optimization Patterns

**Spatial Database:**
- Uniform grid for fast spatial queries in bounded regions
- Stores team and type data to avoid component lookups
- Two query modes: AABB iteration and proximity-ordered iteration

**Planet Navigation Grid:**
- Pre-computed grid for efficient planet avoidance
- Each cell stores closest planet data for O(1) avoidance calculations

**Transform Optimization:**
- LOD entities use `CopyEntityLocalTransformAsLtW` instead of transform hierarchies
- Avoids expensive hierarchy updates for visual-only entities
- ~5x performance improvement over default transform hierarchy

**VFX Optimization:**
- Single VFXGraph per effect type handles all instances
- Uses graphics buffers instead of GameObject instantiation
- Allows thousands of concurrent VFX with minimal performance cost

## File Structure

**Scripts Organization:**
- `/Assets/Scripts/` - Root for all code
- `/Assets/Scripts/Components/` - ECS component definitions
- `/Assets/Scripts/Authoring/` - MonoBehaviour to ECS conversion
- `/Assets/Scripts/Authoring/BlobAuthoring/` - Blob asset authoring classes
- `/Assets/Scripts/Utilities/` - Helper classes and spatial systems
- `/Assets/Scripts/UI/` - UI Toolkit-based interface

**Key Authoring Patterns:**
- Extensive use of `[RequireComponent()]` to ensure component dependencies
- Blob authoring via `IBlobAuthoring` for shared data
- Initialization via `Initialize` enableable component pattern

## Development Guidelines

**ECS Job Patterns:**
- Parallel jobs for computation (AI, navigation, resource updates)
- Single-threaded "Execute" jobs for operations requiring write access to shared data
- Jobs must be Burst-compiled (`[BurstCompile]`) for performance

**System Dependencies:**
- Systems specify update order via `[UpdateAfter]` and `[UpdateBefore]`
- Critical: AI systems must run before ship/building execution systems
- Spatial database must be built before any spatial queries

**Memory Management:**
- Use blob assets for shared immutable data
- Prefer parallel jobs over IJobEntity when possible
- Native collections must be properly disposed

**Debugging:**
- The project includes debug visualization systems
- See `_Documentation/debug-views.md` for visualization options
- Use Debug Views to understand AI decisions and spatial structures