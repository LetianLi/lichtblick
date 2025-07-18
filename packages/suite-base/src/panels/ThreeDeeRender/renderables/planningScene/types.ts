// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import type { Header, Point, Quaternion, TransformStamped, Vector3 } from "../../ros";
import type { Pose } from "../../transforms/geometry";

// MoveIt Planning Scene Message Types
// Based on Noetic moveit_msgs/PlanningScene and related message definitions

export type PlanningScene = {
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
  joint_state: JointState; // Positions of robot joints
  multi_dof_joint_state: MultiDOFJointState; // Multi-DOF joints
  attached_collision_objects: AttachedCollisionObject[]; // Attached collision objects
  is_diff: boolean; // If this state is a diff w.r.t. another state
};

export type JointState = {
  header: Header; // Header for time/frame
  name: string[]; // Joint names
  position: number[]; // Joint positions
  velocity: number[]; // Joint velocities
  effort: number[]; // Joint efforts
};

export type MultiDOFJointState = {
  header: Header; // Header for time/frame
  joint_names: string[]; // Names of multi-DOF joints
  transforms: Transform[]; // Transforms for each joint
  twist: Twist[]; // (Optional) Twist for each joint
  wrench: Wrench[]; // (Optional) Wrench for each joint
};

export type Transform = {
  translation: Vector3; // Translation vector
  rotation: Quaternion; // Rotation quaternion
};

export type Twist = {
  linear: Vector3; // Linear velocity
  angular: Vector3; // Angular velocity
};

export type Wrench = {
  force: Vector3; // Force vector
  torque: Vector3; // Torque vector
};

export type AllowedCollisionMatrix = {
  entry_names: string[]; // Names in the matrix
  entry_values: AllowedCollisionEntry[]; // Matrix entries (square, symmetric)
  default_entry_names: string[]; // Names for default entries
  default_entry_values: boolean[]; // Default values for each entry name
};

export type AllowedCollisionEntry = {
  enabled: boolean[]; // Whether collision checking is enabled for each pair
};

export type LinkPadding = {
  link_name: string; // Name for the link
  padding: number; // Padding to apply to the link
};

export type LinkScale = {
  link_name: string; // Name for the link
  scale: number; // Scaling to apply to the link
};

export type ObjectColor = {
  id: string; // Object id for which color is specified
  color: ColorRGBA; // Color value
};

export type ColorRGBA = {
  r: number; // Red
  g: number; // Green
  b: number; // Blue
  a: number; // Alpha
};

export type PlanningSceneWorld = {
  collision_objects: CollisionObject[]; // Collision objects
  octomap: OctomapWithPose; // Octomap for additional collision data
};

export type CollisionObject = {
  header: Header; // Header for time/frame
  pose: Pose; // The object's pose relative to the header frame
  id: string; // Object id (name used in MoveIt)
  type: ObjectType; // Object type in a database of known objects
  primitives: SolidPrimitive[]; // Solid geometric primitives
  primitive_poses: Pose[]; // Poses for each primitive
  meshes: Mesh[]; // Meshes
  mesh_poses: Pose[]; // Poses for each mesh
  planes: Plane[]; // Bounding planes
  plane_poses: Pose[]; // Poses for each plane
  subframe_names: string[]; // Named subframes on the object
  subframe_poses: Pose[]; // Poses for each subframe
  operation: CollisionObjectOperation; // Operation to be performed
};

// object_recognition_msgs/ObjectType
export type ObjectType = {
  key: string; // Database key
  db: string; // Database name
};

export enum CollisionObjectOperation {
  ADD = 0, // Adds the object to the planning scene (replace if exists)
  REMOVE = 1, // Removes the object from the environment
  APPEND = 2, // Append to an object that already exists
  MOVE = 3, // Move an existing object (geometry arrays must be empty)
}

export type SolidPrimitive = {
  type: SolidPrimitiveType; // Type of primitive
  dimensions: number[]; // Dimensions for the primitive
};

export enum SolidPrimitiveType {
  BOX = 1, // Box
  SPHERE = 2, // Sphere
  CYLINDER = 3, // Cylinder
  CONE = 4, // Cone
  PRISM = 5, // Prism
}

export type Mesh = {
  triangles: MeshTriangle[]; // Mesh triangles
  vertices: Point[]; // Mesh vertices
};

export type MeshTriangle = {
  vertex_indices: [number, number, number]; // Indices of triangle vertices
};

export type Plane = {
  coef: [number, number, number, number]; // Plane equation coefficients
};

export type AttachedCollisionObject = {
  link_name: string; // Link to which the object is attached
  object: CollisionObject; // The actual collision object
  touch_links: string[]; // Links allowed to touch the object
  detach_posture: JointTrajectory; // Posture for releasing the object
  weight: number; // Weight of the attached object
};

// trajectory_msgs/JointTrajectory
export type JointTrajectory = {
  header: Header; // Header for time/frame
  joint_names: string[]; // Names of the joints for this trajectory
  points: JointTrajectoryPoint[]; // Trajectory points
};

// trajectory_msgs/JointTrajectoryPoint
export type JointTrajectoryPoint = {
  // Each trajectory point specifies either positions[, velocities[, accelerations]]
  // or positions[, effort] for the trajectory to be executed.
  // All specified values are in the same order as the joint names in JointTrajectory.
  positions: number[]; // Position values for each joint
  velocities: number[]; // (Optional) Velocity values for each joint
  accelerations: number[]; // (Optional) Acceleration values for each joint
  effort: number[]; // (Optional) Effort values for each joint
  time_from_start: number; // Duration from trajectory start to this point (in seconds)
};

export type OctomapWithPose = {
  header: Header; // Header for time/frame
  origin: Pose; // Pose of the octree w.r.t. header frame
  octomap: Octomap; // The actual octree msg
};

export type Octomap = {
  header: Header; // Header for time/frame
  binary: boolean; // True if binary octree
  id: string; // Class id of the octree
  resolution: number; // Resolution of smallest octree nodes
  data: number[]; // Binary serialization of octree (int8[] in ROS)
};

// Service Types
export type GetPlanningSceneRequest = {
  components: PlanningSceneComponents;
};

export type GetPlanningSceneResponse = {
  scene: PlanningScene;
};

export type PlanningSceneComponents = {
  components: number;
};

// Planning Scene Component Flags
export enum PlanningSceneComponentsFlags {
  SCENE_SETTINGS = 1,
  ROBOT_STATE = 2,
  ROBOT_STATE_ATTACHED_OBJECTS = 4,
  WORLD_OBJECT_NAMES = 8,
  WORLD_OBJECT_GEOMETRY = 16,
  OCTOMAP = 32,
  TRANSFORMS = 64,
  ALLOWED_COLLISION_MATRIX = 128,
  LINK_PADDING_AND_SCALING = 256,
  OBJECT_COLORS = 512,
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
export const DEFAULT_PLANNING_SCENE_TOPIC = "/planning_scene";

// Utility function to create default PlanningSceneComponents for service requests
export function createDefaultPlanningSceneComponents(): PlanningSceneComponents {
  return {
    components:
      PlanningSceneComponentsFlags.SCENE_SETTINGS |
      PlanningSceneComponentsFlags.ROBOT_STATE |
      PlanningSceneComponentsFlags.ROBOT_STATE_ATTACHED_OBJECTS |
      PlanningSceneComponentsFlags.WORLD_OBJECT_NAMES |
      PlanningSceneComponentsFlags.WORLD_OBJECT_GEOMETRY |
      PlanningSceneComponentsFlags.OCTOMAP |
      PlanningSceneComponentsFlags.TRANSFORMS |
      PlanningSceneComponentsFlags.ALLOWED_COLLISION_MATRIX |
      PlanningSceneComponentsFlags.LINK_PADDING_AND_SCALING |
      PlanningSceneComponentsFlags.OBJECT_COLORS,
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
