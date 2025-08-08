// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { Duration } from "@lichtblick/rostime";

import type {
  Header,
  Point,
  TransformStamped,
  Vector3,
  JointState,
  ColorRGBA,
  Transform,
} from "../../ros";
import type { Pose } from "../../transforms/geometry";

// MoveIt Planning Scene Message Types
// Based on Noetic moveit_msgs/PlanningScene and related message definitions

export type PlanningScene = {
  // moveit_msgs/PlanningScene
  name: string; // Name of planning scene
  robot_state: RobotState; // Full robot state
  robot_model_name: string; // Name of the robot model this scene is for
  fixed_frame_transforms: TransformStamped[]; // Additional frames for duplicating tf (w.r.t. planning frame)
  allowed_collision_matrix: AllowedCollisionMatrix; // Full allowed collision matrix
  link_padding: LinkPadding[]; // All link paddings
  link_scale: LinkScale[]; // All link scales
  object_colors: ObjectColor[]; // Colors for attached objects, collision objects, octomap, etc.
  world: PlanningSceneWorld; // The collision map
  is_diff: boolean; // If this scene is a diff w.r.t. another scene
};

export type RobotState = {
  // moveit_msgs/RobotState
  joint_state: JointState; // Positions of robot joints
  multi_dof_joint_state: MultiDOFJointState; // Multi-DOF joints
  attached_collision_objects: AttachedCollisionObject[]; // Attached collision objects
  is_diff: boolean; // If this state is a diff w.r.t. another state. Mostly for handling attached bodies (whether to clear them before updating with this msg)
};

export type MultiDOFJointState = {
  // sensor_msgs/MultiDOFJointState
  header: Header; // Header for time/frame
  joint_names: string[]; // Names of multi-DOF joints
  transforms: Transform[]; // Transforms for each joint, or empty if not applicable
  twist: Twist[]; // Twist for each joint, or empty if not applicable
  wrench: Wrench[]; // Wrench for each joint, or empty if not applicable
};

export type Twist = {
  // geometry_msgs/Twist: Velocity in free space
  linear: Vector3; // Linear velocity
  angular: Vector3; // Angular velocity
};

export type Wrench = {
  // geometry_msgs/Wrench: Force in free space
  force: Vector3; // Force vector
  torque: Vector3; // Torque vector
};

export type AttachedCollisionObject = {
  // moveit_msgs/AttachedCollisionObject
  link_name: string; // Link to which the object is attached
  object: CollisionObject; // The actual collision object with shapes and poses
  touch_links: string[]; // Links allowed to touch the object (link_name is already defaulted)
  detach_posture: JointTrajectory; // Posture for releasing the object
  weight: number; // Weight of the attached object
};

export type CollisionObject = {
  // moveit_msgs/CollisionObject
  header: Header; // Header, used for interpreting the poses
  pose: Pose; // The object's pose relative to the header frame. All shapes and subframes are relative to this pose
  id: string; // Object id (name used in MoveIt)
  type: ObjectType; // Object type in a database of known objects
  primitives: SolidPrimitive[]; // Solid geometric primitives
  primitive_poses: Pose[];
  meshes: Mesh[]; // Meshes
  mesh_poses: Pose[];
  planes: Plane[]; // Bounding planes (equation specified, but can be orientated using additional pose)
  plane_poses: Pose[];
  subframe_names: string[]; // Named subframes on the object. Use to define POIs on object for planning. If an object "screwdriver" and subframe "tip" exist, you can use "screwdriver/tip" for planning.
  subframe_poses: Pose[];
  operation: CollisionObjectOperation; // Operation to be performed
};

export type ObjectType = {
  // object_recognition_msgs/ObjectType
  key: string; // The key of the found object: the unique identifier in the given db
  db: string; // The db parameters stored as a JSON/compressed YAML string.
};

export type SolidPrimitive = {
  // shape_msgs/SolidPrimitive
  type: SolidPrimitiveType; // Type of primitive
  dimensions: Float64Array | number[]; // Dimensions of the shape, with bouding boxes centered around 0,0,0
};

export enum SolidPrimitiveType {
  BOX = 1, // Box
  SPHERE = 2, // Sphere
  CYLINDER = 3, // Cylinder
  CONE = 4, // Cone
}

/* eslint-disable @typescript-eslint/no-duplicate-enum-values */
export enum SolidPrimitiveDimension {
  BOX_X = 0,
  BOX_Y = 1,
  BOX_Z = 2,
  SPHERE_RADIUS = 0,
  CYLINDER_HEIGHT = 0,
  CYLINDER_RADIUS = 1,
  CONE_HEIGHT = 0, // Cone tip points up along the +Z axis
  CONE_RADIUS = 1,
}
/* eslint-enable @typescript-eslint/no-duplicate-enum-values */

export type Mesh = {
  // shape_msgs/Mesh
  triangles: MeshTriangle[]; // Mesh triangles; Index values refer to positions in vertices array
  vertices: Point[]; // The actual vertices of the mesh
};

export type MeshTriangle = {
  // shape_msgs/MeshTriangle
  vertex_indices: Uint32Array; // Definition of a triangle's vertices. Indices refer to parent vertices array.
};

export type Plane = {
  // shape_msgs/Plane
  coef: [number, number, number, number] | (Float64Array & { length: 4 }); // Plane equation coefficients: ax + by + cz + d = 0
};

export enum CollisionObjectOperation {
  ADD = 0, // Adds the object to the planning scene (replace if exists)
  REMOVE = 1, // Removes the object from the environment (all that match the id)
  APPEND = 2, // Append to an object that already exists (adds if it doesn't exist)
  MOVE = 3, // Move an existing object (geometry arrays must be empty)
}

export type JointTrajectory = {
  // trajectory_msgs/JointTrajectory
  header: Header; // Header for time/frame
  joint_names: string[]; // Names of the joints for this trajectory
  points: JointTrajectoryPoint[]; // Trajectory points
};

export type JointTrajectoryPoint = {
  // trajectory_msgs/JointTrajectoryPoint
  positions: Float64Array | number[]; // Position for each joint at this trajectory waypoint
  velocities: Float64Array | number[]; // Velocity for each joint, or empty if not used
  accelerations: Float64Array | number[]; // Acceleration for each joint, or empty if not used
  effort: Float64Array | number[]; // Effort for each joint, or empty if not used
  time_from_start: Duration; // Time from start of trajectory to reach this point
};

export type AllowedCollisionMatrix = {
  // moveit_msgs/AllowedCollisionMatrix
  entry_names: string[]; // List of entry names in the matrix
  entry_values: AllowedCollisionEntry[]; // Individual entries (should be square, symmetric, and same order)
  default_entry_names: string[]; // Names for default entries.
  default_entry_values: boolean[]; // Default values for each entry name. Used if name(s) could only be found in default names.
};

export type AllowedCollisionEntry = {
  // moveit_msgs/AllowedCollisionEntry
  enabled: boolean[]; // Whether collision checking is enabled
};

export type LinkPadding = {
  // moveit_msgs/LinkPadding
  link_name: string; // Name for the link
  padding: number; // Padding to apply to the link
};

export type LinkScale = {
  // moveit_msgs/LinkScale
  link_name: string; // Name for the link
  scale: number; // Scaling to apply to the link
};

export type ObjectColor = {
  // moveit_msgs/ObjectColor
  id: string; // Object id for which color is specified
  color: ColorRGBA; // Color value
};

export type PlanningSceneWorld = {
  // moveit_msgs/PlanningSceneWorld
  collision_objects: CollisionObject[]; // Collision objects
  octomap: OctomapWithPose; // Octomap representing additional collision data
};

export type OctomapWithPose = {
  // octomap_msgs/OctomapWithPose: A 3D map in binary format, as Octree
  header: Header;
  origin: Pose; // Pose of the octree w.r.t. header frame
  octomap: Octomap; // The actual octree msg
};

export type Octomap = {
  // octomap_msgs/Octomap: A 3D map in binary format, as Octree
  header: Header;
  binary: boolean; // Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
  id: string; // Class id of the contained octree
  resolution: number; // Resolution (in meters) of smallest octree nodes
  data: Int8Array | number[]; // Binary serialization of octree, use conversions.h to read and write octrees
};

// Service Types
export type GetPlanningSceneRequest = {
  // moveit_msgs/GetPlanningScene
  components: PlanningSceneComponents;
};

export type GetPlanningSceneResponse = {
  // moveit_msgs/GetPlanningScene
  scene: PlanningScene;
};

export type PlanningSceneComponents = {
  // moveit_msgs/PlanningSceneComponents
  components: number; // uint32 bitmask of PlanningSceneComponentsMask values specifying which parts of the PlanningScene message are of interest (0 fetches all)
};

export enum PlanningSceneComponentsMask {
  SCENE_SETTINGS = 1, // Scene name, model name, model root
  ROBOT_STATE = 2, // Joint values of the robot state
  ROBOT_STATE_ATTACHED_OBJECTS = 4, // Attached objects (including geometry) for the robot state
  WORLD_OBJECT_NAMES = 8, // The names of the world objects
  WORLD_OBJECT_GEOMETRY = 16, // The geometry of the world objects
  OCTOMAP = 32, // The maintained octomap
  TRANSFORMS = 64, // The maintained list of transforms
  ALLOWED_COLLISION_MATRIX = 128, // The allowed collision matrix
  LINK_PADDING_AND_SCALING = 256, // The default link padding and link scaling
  OBJECT_COLORS = 512, // The stored object colors
}

// Message schema constants for topic subscription filtering
export const PLANNING_SCENE_DATATYPES = new Set<string>([
  "moveit_msgs/PlanningScene",
  "moveit_msgs/msg/PlanningScene",
]);

export const GET_PLANNING_SCENE_DATATYPES = new Set<string>([
  "moveit_msgs/GetPlanningScene",
  "moveit_msgs/srv/GetPlanningScene",
]);

// Default service and topic names
export const DEFAULT_PLANNING_SCENE_SERVICE = "/get_planning_scene";
export const DEFAULT_PLANNING_SCENE_TOPIC = "/move_group/monitored_planning_scene"; // This is used internally for reading by RViz. "/planning_scene" may not capture all updates.

// Utility function to create minimal PlanningSceneComponents for service requests
// Only requests the components we actually use to reduce data transfer and processing
export function createMinimalPlanningSceneComponents(): PlanningSceneComponents {
  return {
    components:
      PlanningSceneComponentsMask.SCENE_SETTINGS |
      // PlanningSceneComponentsMask.ROBOT_STATE |
      PlanningSceneComponentsMask.ROBOT_STATE_ATTACHED_OBJECTS |
      PlanningSceneComponentsMask.WORLD_OBJECT_NAMES |
      PlanningSceneComponentsMask.WORLD_OBJECT_GEOMETRY |
      // PlanningSceneComponentsMask.OCTOMAP |
      // PlanningSceneComponentsMask.TRANSFORMS |
      // PlanningSceneComponentsMask.ALLOWED_COLLISION_MATRIX |
      // PlanningSceneComponentsMask.LINK_PADDING_AND_SCALING |
      PlanningSceneComponentsMask.OBJECT_COLORS,
  };
}

// Utility function to check if a topic schema matches planning scene types
export function isPlanningSceneTopic(schemaName: string): boolean {
  return PLANNING_SCENE_DATATYPES.has(schemaName);
}

// Utility function to check if a service schema matches GetPlanningScene type
export function isGetPlanningSceneService(schemaName: string): boolean {
  return GET_PLANNING_SCENE_DATATYPES.has(schemaName);
}

// Normalization functions to handle arrays from ROS messages
export function normalizeVertexIndices(indices: unknown): number[] {
  if (indices == undefined) {
    return [];
  }
  if (Array.isArray(indices)) {
    return indices.map(Number);
  }
  if (indices instanceof Uint32Array || indices instanceof Int32Array) {
    return Array.from(indices);
  }
  // Handle other typed arrays or array-like objects
  if (typeof indices === "object" && "length" in indices) {
    return Array.from(indices as ArrayLike<number>).map(Number);
  }
  return [];
}

export function normalizeDimensions(dimensions: unknown): number[] {
  if (dimensions == undefined) {
    return [];
  }
  if (Array.isArray(dimensions)) {
    return dimensions.map(Number);
  }
  if (dimensions instanceof Float32Array || dimensions instanceof Float64Array) {
    return Array.from(dimensions);
  }
  // Handle other typed arrays or array-like objects
  if (typeof dimensions === "object" && "length" in dimensions) {
    return Array.from(dimensions as ArrayLike<number>).map(Number);
  }
  return [];
}

/**
 * Get the dimension names for a solid primitive type, useful for error messages and documentation
 */
export function getSolidPrimitiveDimensionNames(primitiveType: SolidPrimitiveType): string[] {
  switch (primitiveType) {
    case SolidPrimitiveType.BOX:
      return ["x", "y", "z"];
    case SolidPrimitiveType.SPHERE:
      return ["radius"];
    case SolidPrimitiveType.CYLINDER:
      return ["height", "radius"];
    case SolidPrimitiveType.CONE:
      return ["height", "radius"];
    default:
      return ["dimensions"];
  }
}
