# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Unity 6 project using DOTS (Data-Oriented Technology Stack) that implements a large-scale space simulation game called "Galaxy Sample". The game simulates teams of spaceships fighting for control of planets using Unity's Entity Component System (ECS).

**Key Features:**
- Full 3D flight simulation with aerial combat
- Octree-based spatial database for efficient collision detection
- Flow field navigation system for high-speed obstacle avoidance
- Modular steering behavior system with flocking, chase, flee, and wander behaviors
- Support for complex mesh colliders and compound geometry
- High-performance parallel processing with Burst compilation

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
2. **FlowFieldGenerationSystemOptimized** - Generates 3D flow fields for navigation around complex obstacles
3. **BuildOctreeSpatialDatabaseSystem** - Populates octree spatial database with entities for collision detection
4. **TeamAISystem** - Computes AI actions and importance scores for all entities
5. **SteeringBehaviorSystem** - Advanced AI with modular weighted behaviors (seek, flee, flock, wander, flow field)
6. **HighSpeedChaseSystem** - Specialized AI for aerial combat and high-speed navigation
7. **ShipSystem** - Core ship logic (navigation, AI decisions, combat)
8. **BuildingSystem** - Factory production, turret combat, research bonuses
9. **PlanetSystem** - Resource generation, conversion, ship assessment
10. **ComplexAvoidanceSystem** - Enhanced obstacle avoidance using mesh colliders
11. **VFXSystem** - High-performance VFX using graphics buffers instead of GameObjects

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

**Navigation & AI Components:**
- `SteeringBehaviors` - Modular weighted steering system with configurable behavior weights
- `FlockMember` - Identifies entities that can participate in flocking behaviors
- `HighSpeedChaseAI` - Specialized high-speed aerial combat AI
- `ComplexAvoidanceData` - Settings for advanced obstacle avoidance using mesh colliders

**Spatial Database Components:**
- `OctreeSpatialDatabase` - Root octree spatial database entity
- `OctreeNode` - Individual octree nodes for hierarchical spatial partitioning
- `SpatialObject` - Objects stored in spatial database with collision shape information

**Flow Field Components:**
- `FlowField` - 3D grid-based navigation field with different types (Chase, Flee, Patrol, Ambush)
- `FlowCell` - Individual cells in flow field containing velocity vectors and obstacle distances
- `FlowFieldCollection` - Collection of multiple flow fields for blended navigation
- `FlowFieldGenerationControl` - Controls when flow fields are regenerated based on target movement

### Blob Asset Data Pattern

The project uses blob assets for shared, immutable data:
- `ShipData`, `FighterData`, `WorkerData`, `TraderData` for ship types
- `BuildingData`, `TurretData` for buildings
- Implemented via `IBlobAuthoring<T>` interface in `BlobAuthoring` utility classes
- Reduces memory footprint and improves chunk layout performance

### AI System Architecture

**Multi-Layer AI System:**

**1. Strategic AI (TeamAISystem):**
- Computes empire-wide statistics (planets, ships, resources)
- Generates action lists with importance scores:
  - `FighterAction` - Which planets to attack/defend
  - `WorkerAction` - Which planets to capture or buildings to construct
  - `TraderAction` - Resource distribution routes
  - `FactoryAction` - Which ship types to produce
- Individual entities choose actions via weighted random selection with personal bias

**2. Tactical AI (SteeringBehaviorSystem):**
- Modular weighted steering behaviors that can run simultaneously:
  - **Seek/Flee** - Move toward or away from specific targets
  - **Flocking** - Cohesion, separation, and alignment with nearby units
  - **Wander** - Random exploration behavior
  - **Flow Field Following** - High-performance navigation around complex obstacles
  - **Obstacle Avoidance** - Real-time collision avoidance using spatial queries
  - **Leader Following** - Formation flying and escort behaviors
- All behaviors blend together using configurable weights (0.0 = off, 2.0 = double strength)

**3. High-Speed AI (HighSpeedChaseSystem):**
- Specialized for aerial combat scenarios with fast-moving targets
- State machine with behaviors: Idle, Chase, Flee, Ambush, Patrol, LoseTarget
- Integrates flow field navigation with predictive targeting
- Optimized for high-speed maneuvers around complex obstacles

### Performance Optimization Patterns

**Octree Spatial Database:**
- Hierarchical spatial partitioning for efficient collision detection and proximity queries
- Adaptive subdivision based on entity density
- Support for multiple collision shape types: Point, AABB, Sphere, Mesh, Compound, ConvexHull
- O(log n) spatial queries vs O(n) brute force
- Optimized octree traversal with early exit conditions and distance culling

**Flow Field Navigation:**
- Pre-computed 3D navigation grids for high-performance obstacle avoidance
- Four field types: Chase, Flee, Patrol, Ambush with blendable weights
- Parallel SDF (Signed Distance Field) generation using Burst-compiled jobs
- 26-neighbor connectivity for full 3D flight with diagonal movement support
- Adaptive regeneration based on target movement and time intervals
- Memory-efficient: ~2MB per 64×32×64 flow field

**3D Flight Optimizations:**
- Full 3D gradient calculation for up/down/diagonal movement
- Multi-altitude goal placement for aerial escape routes and ambush tactics
- Predictive targeting for high-speed intercept scenarios

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
- `/Assets/Scripts/AI/` - Advanced AI systems (steering behaviors, high-speed chase)
- `/Assets/Scripts/Utilities/` - Helper classes and spatial systems
- `/Assets/Scripts/Utilities/SpatialDatabase/` - Octree spatial database implementation
- `/Assets/Scripts/UI/` - UI Toolkit-based interface

**Key Authoring Patterns:**
- Extensive use of `[RequireComponent()]` to ensure component dependencies
- Blob authoring via `IBlobAuthoring` for shared data
- Initialization via `Initialize` enableable component pattern

**Navigation & AI Authoring:**
- `FlowFieldAuthoring` - Configure 3D flow fields with resolution, bounds, and target assignments
- `FlowFieldCollectionAuthoring` - Manage multiple flow fields for blended navigation
- `SteeringBehaviorAuthoring` - Unity Inspector interface for configuring behavior weights and parameters
- `ComplexAvoidanceAuthoring` - Settings for mesh collider avoidance with different detail levels

**Spatial Database Authoring:**
- `OctreeSpatialDatabaseAuthoring` - Configure octree parameters and visualization
- `SpatialDatabaseSingleton` - Global reference to active spatial database
- Prefab-based configuration for scene-specific spatial database settings

## Development Guidelines

**ECS Job Patterns:**
- Parallel jobs for computation (AI, navigation, resource updates)
- Single-threaded "Execute" jobs for operations requiring write access to shared data
- Jobs must be Burst-compiled (`[BurstCompile]`) for performance

**System Dependencies:**
- Systems specify update order via `[UpdateAfter]` and `[UpdateBefore]`
- Critical: AI systems must run before ship/building execution systems
- Octree spatial database must be built before any spatial queries
- Flow field generation must complete before navigation systems
- Steering behaviors integrate multiple systems: flow fields, spatial database, and flocking

**Memory Management:**
- Use blob assets for shared immutable data
- Prefer parallel jobs over IJobEntity when possible
- Native collections must be properly disposed
- Flow fields can use significant memory (~2MB per 64×32×64 grid)
- Octree nodes and spatial objects are stored in DynamicBuffers for efficiency

**3D Flight Development:**
- Always consider full 3D movement - ships can fly up, down, and diagonally
- Use 26-neighbor connectivity in pathfinding and flow fields
- Design AI behaviors for aerial combat scenarios (chase, flee, ambush)
- Test navigation around complex 3D obstacles like floating islands

**Performance Considerations:**
- Flow field generation is CPU-intensive - use adaptive regeneration
- Octree spatial queries are O(log n) but can still be expensive with many objects
- Steering behaviors should be weighted appropriately to avoid conflicting forces
- Use LOD systems for distant entities to reduce computational load

**Debugging:**
- The project includes debug visualization systems
- See `_Documentation/debug-views.md` for visualization options
- Use Debug Views to understand AI decisions and spatial structures
- Flow fields can be visualized using DrawOctreeNode methods in authoring components
- Gizmos show octree bounds, flow field bounds, and navigation paths