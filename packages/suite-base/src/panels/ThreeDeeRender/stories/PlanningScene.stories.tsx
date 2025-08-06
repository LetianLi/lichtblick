// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { StoryObj } from "@storybook/react";

import { MessageEvent } from "@lichtblick/suite";
import { Topic } from "@lichtblick/suite-base/players/types";
import PanelSetup from "@lichtblick/suite-base/stories/PanelSetup";
import useDelayedFixture from "./useDelayedFixture";

import ThreeDeePanel from "../index";
import type {
  PlanningScene as PlanningSceneType,
  CollisionObject,
  SolidPrimitive,
} from "../renderables/planningScene/types";
import {
  SolidPrimitiveType,
  CollisionObjectOperation,
} from "../renderables/planningScene/types";
import type { Header, TransformStamped } from "../ros";
import type { Pose } from "../transforms";
import { makeColor, QUAT_IDENTITY, rad2deg, SENSOR_FRAME_ID } from "./common";

export default {
  title: "panels/ThreeDeeRender",
  component: ThreeDeePanel,
  parameters: {
    colorScheme: "dark",
    chromatic: { delay: 100 },
  },
};

const DEFAULT_PLANNING_SCENE_TOPIC = "/move_group/monitored_planning_scene";

// Helper function to create a pose
function makePose(x: number, y: number, z: number, qx = 0, qy = 0, qz = 0, qw = 1): Pose {
  return {
    position: { x, y, z },
    orientation: { x: qx, y: qy, z: qz, w: qw },
  };
}

// Helper function to create a header
function makeHeader(frameId = "map"): Header {
  return {
    stamp: { sec: 0, nsec: 0 },
    frame_id: frameId,
  };
}

// Helper function to create a solid primitive
function makeSolidPrimitive(type: SolidPrimitiveType, dimensions: number[]): SolidPrimitive {
  return {
    type,
    dimensions,
  };
}

// Helper function to create a collision object
function makeCollisionObject(
  id: string,
  primitives: SolidPrimitive[],
  primitivePoses: Pose[],
  pose: Pose = makePose(0, 0, 0),
  operation: CollisionObjectOperation = CollisionObjectOperation.ADD,
): CollisionObject {
  return {
    header: makeHeader(),
    id,
    pose,
    type: { key: "", db: "" },
    primitives,
    primitive_poses: primitivePoses,
    meshes: [],
    mesh_poses: [],
    planes: [],
    plane_poses: [],
    subframe_names: [],
    subframe_poses: [],
    operation,
  };
}

// Create synthetic planning scene with various collision objects arranged in a grid
function createSyntheticPlanningScene(): PlanningSceneType {
  const collisionObjects: CollisionObject[] = [
    // Row 1: Successful primitives (0-6)
    // Object 0: Box at (0, 0, 1) - No rotation
    makeCollisionObject(
      "test_box_1",
      [makeSolidPrimitive(SolidPrimitiveType.BOX, [1.0, 1.0, 1.0])],
      [makePose(0.0, 0.0, 1.0)],
      makePose(0, 0, 0),
    ),

    // Object 1: Sphere at (2, 0, 1) - 22.5° rotation around X
    makeCollisionObject(
      "test_sphere_1",
      [makeSolidPrimitive(SolidPrimitiveType.SPHERE, [1.0])],
      [makePose(2.0, 0.0, 1.0, 0.195, 0, 0, 0.981)],
      makePose(0, 0, 0),
    ),

    // Object 2: Cylinder at (4, 0, 1) - 22.5° rotation around Y
    makeCollisionObject(
      "test_cylinder_1",
      [makeSolidPrimitive(SolidPrimitiveType.CYLINDER, [2.0, 1.0])],
      [makePose(4.0, 0.0, 1.0, 0, 0.195, 0, 0.981)],
      makePose(0, 0, 0),
    ),

    // Object 3: Cone at (6, 0, 1) - 22.5° rotation around Z
    makeCollisionObject(
      "test_cone_1",
      [makeSolidPrimitive(SolidPrimitiveType.CONE, [2.0, 1.0])],
      [makePose(6.0, 0.0, 1.0, 0, 0, 0.195, 0.981)],
      makePose(0, 0, 0),
    ),

    // Object 4: Box at (8, 0, 1) - 45° rotation around X
    makeCollisionObject(
      "test_box_2",
      [makeSolidPrimitive(SolidPrimitiveType.BOX, [1.0, 1.0, 1.0])],
      [makePose(8.0, 0.0, 1.0, 0.383, 0, 0, 0.924)],
      makePose(0, 0, 0),
    ),

    // Object 5: Sphere at (10, 0, 1) - 45° rotation around Y
    makeCollisionObject(
      "test_sphere_2",
      [makeSolidPrimitive(SolidPrimitiveType.SPHERE, [1.0])],
      [makePose(10.0, 0.0, 1.0, 0, 0.383, 0, 0.924)],
      makePose(0, 0, 0),
    ),

    // Object 6: Cylinder at (12, 0, 1) - 45° rotation around Z
    makeCollisionObject(
      "test_cylinder_2",
      [makeSolidPrimitive(SolidPrimitiveType.CYLINDER, [2.0, 1.0])],
      [makePose(12.0, 0.0, 1.0, 0, 0, 0.383, 0.924)],
      makePose(0, 0, 0),
    ),

    // Row 2: Complex rotations (7-13)
    // Object 7: Box at (0, 2, 1) - Combined rotation (X=30°, Y=45°, Z=60°)
    makeCollisionObject(
      "test_box_complex_1",
      [makeSolidPrimitive(SolidPrimitiveType.BOX, [1.0, 1.0, 1.0])],
      [makePose(0.0, 2.0, 1.0, 0.259, 0.354, 0.612, 0.659)],
      makePose(0, 0, 0),
    ),

    // Object 8: Sphere at (2, 2, 1) - Roll-Pitch-Yaw (30°, 45°, 60°)
    makeCollisionObject(
      "test_sphere_complex_1",
      [makeSolidPrimitive(SolidPrimitiveType.SPHERE, [1.0])],
      [makePose(2.0, 2.0, 1.0, 0.259, 0.354, 0.612, 0.659)],
      makePose(0, 0, 0),
    ),

    // Object 9: Cylinder at (4, 2, 1) - Identity rotation
    makeCollisionObject(
      "test_cylinder_identity",
      [makeSolidPrimitive(SolidPrimitiveType.CYLINDER, [2.0, 1.0])],
      [makePose(4.0, 2.0, 1.0, 0, 0, 0, 1)],
      makePose(0, 0, 0),
    ),

    // Object 10: Cone at (6, 2, 1) - 90° rotation around X
    makeCollisionObject(
      "test_cone_90_x",
      [makeSolidPrimitive(SolidPrimitiveType.CONE, [2.0, 1.0])],
      [makePose(6.0, 2.0, 1.0, 0.707, 0, 0, 0.707)],
      makePose(0, 0, 0),
    ),

    // Object 11: Box at (8, 2, 1) - 90° rotation around Y
    makeCollisionObject(
      "test_box_90_y",
      [makeSolidPrimitive(SolidPrimitiveType.BOX, [1.0, 1.0, 1.0])],
      [makePose(8.0, 2.0, 1.0, 0, 0.707, 0, 0.707)],
      makePose(0, 0, 0),
    ),

    // Object 12: Sphere at (10, 2, 1) - 90° rotation around Z
    makeCollisionObject(
      "test_sphere_90_z",
      [makeSolidPrimitive(SolidPrimitiveType.SPHERE, [1.0])],
      [makePose(10.0, 2.0, 1.0, 0, 0, 0.707, 0.707)],
      makePose(0, 0, 0),
    ),

    // Object 13: Cylinder at (12, 2, 1) - 180° rotation around X
    makeCollisionObject(
      "test_cylinder_180_x",
      [makeSolidPrimitive(SolidPrimitiveType.CYLINDER, [2.0, 1.0])],
      [makePose(12.0, 2.0, 1.0, 1, 0, 0, 0)],
      makePose(0, 0, 0),
    ),

    // Row 3: Edge cases and special rotations (14-20)
    // Object 14: Box at (0, 4, 1) - Very small rotation (1° around X)
    makeCollisionObject(
      "test_box_small_rot",
      [makeSolidPrimitive(SolidPrimitiveType.BOX, [1.0, 1.0, 1.0])],
      [makePose(0.0, 4.0, 1.0, 0.009, 0, 0, 1.000)],
      makePose(0, 0, 0),
    ),

    // Object 15: Sphere at (2, 4, 1) - Large rotation (170° around Y)
    makeCollisionObject(
      "test_sphere_large_rot",
      [makeSolidPrimitive(SolidPrimitiveType.SPHERE, [1.0])],
      [makePose(2.0, 4.0, 1.0, 0, 0.985, 0, 0.174)],
      makePose(0, 0, 0),
    ),

    // Object 16: Cylinder at (4, 4, 1) - Gimbal lock case (90° pitch)
    makeCollisionObject(
      "test_cylinder_gimbal_lock",
      [makeSolidPrimitive(SolidPrimitiveType.CYLINDER, [2.0, 1.0])],
      [makePose(4.0, 4.0, 1.0, 0.707, 0, 0, 0.707)],
      makePose(0, 0, 0),
    ),

    // Object 17: Cone at (6, 4, 1) - Negative rotation (-45° around Z)
    makeCollisionObject(
      "test_cone_negative_rot",
      [makeSolidPrimitive(SolidPrimitiveType.CONE, [2.0, 1.0])],
      [makePose(6.0, 4.0, 1.0, 0, 0, -0.383, 0.924)],
      makePose(0, 0, 0),
    ),

    // Object 18: Box at (8, 4, 1) - Multiple rotations (X=15°, Y=30°, Z=45°)
    makeCollisionObject(
      "test_box_multi_rot",
      [makeSolidPrimitive(SolidPrimitiveType.BOX, [1.0, 1.0, 1.0])],
      [makePose(8.0, 4.0, 1.0, 0.131, 0.259, 0.383, 0.876)],
      makePose(0, 0, 0),
    ),

    // Object 19: Sphere at (10, 4, 1) - Quaternion with small values
    makeCollisionObject(
      "test_sphere_small_quat",
      [makeSolidPrimitive(SolidPrimitiveType.SPHERE, [1.0])],
      [makePose(10.0, 4.0, 1.0, 0.001, 0.002, 0.003, 1.000)],
      makePose(0, 0, 0),
    ),

    // Object 20: Cylinder at (12, 4, 1) - Quaternion with large values
    makeCollisionObject(
      "test_cylinder_large_quat",
      [makeSolidPrimitive(SolidPrimitiveType.CYLINDER, [2.0, 1.0])],
      [makePose(12.0, 4.0, 1.0, 0.5, 0.6, 0.7, 0.1)],
      makePose(0, 0, 0),
    ),
  ];

  return {
    name: "Synthetic Planning Scene",
    robot_state: {
      joint_state: {
        header: makeHeader(),
        name: [],
        position: [],
        velocity: [],
        effort: [],
      },
      multi_dof_joint_state: {
        header: makeHeader(),
        joint_names: [],
        transforms: [],
        twist: [],
        wrench: [],
      },
      attached_collision_objects: [],
      is_diff: false,
    },
    robot_model_name: "test_robot",
    fixed_frame_transforms: [],
    allowed_collision_matrix: {
      entry_names: [],
      entry_values: [],
      default_entry_names: [],
      default_entry_values: [],
    },
    link_padding: [],
    link_scale: [],
    object_colors: [],
    world: {
      collision_objects: collisionObjects,
      octomap: {
        header: makeHeader(),
        origin: makePose(0, 0, 0),
        octomap: {
          header: makeHeader(),
          binary: false,
          id: "",
          resolution: 0.1,
          data: [],
        },
      },
    },
    is_diff: false,
  };
}

function PlanningSceneStory(props: { includeSettings?: boolean }): React.JSX.Element {
  const topics: Topic[] = [
    { name: "/tf", schemaName: "geometry_msgs/TransformStamped" },
    { name: DEFAULT_PLANNING_SCENE_TOPIC, schemaName: "moveit_msgs/PlanningScene" },
  ];

  // Create a mock service client that returns an empty planning scene initially
  const mockServiceClient = async (service: string, request: unknown) => {
    if (service === "moveit_msgs/GetPlanningScene" || service === "/get_planning_scene") {
      return {
        scene: {
          name: "Empty Scene",
          robot_state: {
            joint_state: {
              header: makeHeader(),
              name: [],
              position: [],
              velocity: [],
              effort: [],
            },
            multi_dof_joint_state: {
              header: makeHeader(),
              joint_names: [],
              transforms: [],
              twist: [],
              wrench: [],
            },
            attached_collision_objects: [],
            is_diff: false,
          },
          robot_model_name: "test_robot",
          fixed_frame_transforms: [],
          allowed_collision_matrix: {
            entry_names: [],
            entry_values: [],
            default_entry_names: [],
            default_entry_values: [],
          },
          link_padding: [],
          link_scale: [],
          object_colors: [],
          world: {
            collision_objects: [],
            octomap: {
              header: makeHeader(),
              origin: makePose(0, 0, 0),
              octomap: {
                header: makeHeader(),
                binary: false,
                id: "",
                resolution: 0.1,
                data: [],
              },
            },
          },
          is_diff: false,
        },
      };
    }
    throw new Error(`Unknown service: ${service}`);
  };

  const tf1: MessageEvent<TransformStamped> = {
    topic: "/tf",
    receiveTime: { sec: 10, nsec: 0 },
    message: {
      header: { seq: 0, stamp: { sec: 0, nsec: 0 }, frame_id: "map" },
      child_frame_id: "base_link",
      transform: {
        translation: { x: 1e7, y: 0, z: 0 },
        rotation: QUAT_IDENTITY,
      },
    },
    schemaName: "geometry_msgs/TransformStamped",
    sizeInBytes: 0,
  };

  const tf2: MessageEvent<TransformStamped> = {
    topic: "/tf",
    receiveTime: { sec: 10, nsec: 0 },
    message: {
      header: { seq: 0, stamp: { sec: 0, nsec: 0 }, frame_id: "base_link" },
      child_frame_id: SENSOR_FRAME_ID,
      transform: {
        translation: { x: 0, y: 0, z: 0 },
        rotation: QUAT_IDENTITY,
      },
    },
    schemaName: "geometry_msgs/TransformStamped",
    sizeInBytes: 0,
  };

  const planningSceneMessage: MessageEvent<PlanningSceneType> = {
    topic: DEFAULT_PLANNING_SCENE_TOPIC,
    receiveTime: { sec: 10, nsec: 0 },
    message: createSyntheticPlanningScene(),
    schemaName: "moveit_msgs/PlanningScene",
    sizeInBytes: 0,
  };

  const fixture = useDelayedFixture({
    topics,
    frame: {
      "/tf": [tf1, tf2],
      [DEFAULT_PLANNING_SCENE_TOPIC]: [planningSceneMessage],
    },
    capabilities: ["callServices"],
    activeData: {
      currentTime: { sec: 10, nsec: 0 },
    },
    callService: mockServiceClient,
  });

  return (
    <PanelSetup fixture={fixture} includeSettings={props.includeSettings}>
      <ThreeDeePanel
        overrideConfig={{
          ...ThreeDeePanel.defaultConfig,
          followTf: "base_link",
          layers: {
            grid: { layerId: "foxglove.Grid" },
            planningScene: {
              layerId: "foxglove.PlanningScene",
              topic: DEFAULT_PLANNING_SCENE_TOPIC,
              defaultColor: "#ffffff",
              sceneOpacity: 1.0,
              showCollisionObjects: true,
              showAttachedObjects: true,
              showOctomap: false,
            },
          },
          cameraState: {
            distance: 5.5,
            perspective: true,
            phi: rad2deg(0.5),
            targetOffset: [-0.5, 0.75, 0],
            thetaOffset: rad2deg(-0.25),
            fovy: rad2deg(0.75),
            near: 0.01,
            far: 5000,
            target: [0, 0, 0],
            targetOrientation: [0, 0, 0, 1],
          },
          topics: {
            [DEFAULT_PLANNING_SCENE_TOPIC]: { visible: true },
          },
        }}
      />
    </PanelSetup>
  );
}

export const PlanningScene: StoryObj = {
  render: function Story() {
    return <PlanningSceneStory />;
  },
};

export const PlanningSceneSettings: StoryObj = {
  render: function Story() {
    return <PlanningSceneStory includeSettings />;
  },
};

export const PlanningSceneWithErrors: StoryObj = {
  render: function Story() {
    const topics: Topic[] = [
      { name: "/tf", schemaName: "geometry_msgs/TransformStamped" },
      { name: DEFAULT_PLANNING_SCENE_TOPIC, schemaName: "moveit_msgs/PlanningScene" },
    ];

    // Create a mock service client that returns an empty planning scene initially
    const mockServiceClient = async (service: string, request: unknown) => {
      if (service === "moveit_msgs/GetPlanningScene" || service === "/get_planning_scene") {
        return {
          scene: {
            name: "Empty Scene",
            robot_state: {
              joint_state: {
                header: makeHeader(),
                name: [],
                position: [],
                velocity: [],
                effort: [],
              },
              multi_dof_joint_state: {
                header: makeHeader(),
                joint_names: [],
                transforms: [],
                twist: [],
                wrench: [],
              },
              attached_collision_objects: [],
              is_diff: false,
            },
            robot_model_name: "test_robot",
            fixed_frame_transforms: [],
            allowed_collision_matrix: {
              entry_names: [],
              entry_values: [],
              default_entry_names: [],
              default_entry_values: [],
            },
            link_padding: [],
            link_scale: [],
            object_colors: [],
            world: {
              collision_objects: [],
              octomap: {
                header: makeHeader(),
                origin: makePose(0, 0, 0),
                octomap: {
                  header: makeHeader(),
                  binary: false,
                  id: "",
                  resolution: 0.1,
                  data: [],
                },
              },
            },
            is_diff: false,
          },
        };
      }
      throw new Error(`Unknown service: ${service}`);
    };

    const tf1: MessageEvent<TransformStamped> = {
      topic: "/tf",
      receiveTime: { sec: 10, nsec: 0 },
      message: {
        header: { seq: 0, stamp: { sec: 0, nsec: 0 }, frame_id: "map" },
        child_frame_id: "base_link",
        transform: {
          translation: { x: 1e7, y: 0, z: 0 },
          rotation: QUAT_IDENTITY,
        },
      },
      schemaName: "geometry_msgs/TransformStamped",
      sizeInBytes: 0,
    };

    const tf2: MessageEvent<TransformStamped> = {
      topic: "/tf",
      receiveTime: { sec: 10, nsec: 0 },
      message: {
        header: { seq: 0, stamp: { sec: 0, nsec: 0 }, frame_id: "base_link" },
        child_frame_id: SENSOR_FRAME_ID,
        transform: {
          translation: { x: 0, y: 0, z: 0 },
          rotation: QUAT_IDENTITY,
        },
      },
      schemaName: "geometry_msgs/TransformStamped",
      sizeInBytes: 0,
    };

    // Create a planning scene with some invalid data to test error handling
    const invalidPlanningScene: PlanningSceneType = {
      ...createSyntheticPlanningScene(),
      world: {
        ...createSyntheticPlanningScene().world,
        collision_objects: [
          // Valid object
          makeCollisionObject(
            "valid_object",
            [makeSolidPrimitive(SolidPrimitiveType.BOX, [1.0, 1.0, 1.0])],
            [makePose(0, 0, 1.0)],
          ),
          // Object with undefined primitive (should cause error)
          makeCollisionObject(
            "invalid_object_undefined",
            [undefined as any],
            [makePose(1, 0, 1.0)],
          ),
          // Object with invalid dimensions (should cause error)
          makeCollisionObject(
            "invalid_object_dimensions",
            [makeSolidPrimitive(SolidPrimitiveType.BOX, [-1.0, 0, 1.0])], // Negative dimension
            [makePose(2, 0, 1.0)],
          ),
          // Object with invalid pose (should cause error)
          makeCollisionObject(
            "invalid_object_pose",
            [makeSolidPrimitive(SolidPrimitiveType.SPHERE, [1.0])],
            [makePose(NaN, 0, 1.0)], // Invalid position
          ),
        ],
      },
    };

    const planningSceneMessage: MessageEvent<PlanningSceneType> = {
      topic: DEFAULT_PLANNING_SCENE_TOPIC,
      receiveTime: { sec: 10, nsec: 0 },
      message: invalidPlanningScene,
      schemaName: "moveit_msgs/PlanningScene",
      sizeInBytes: 0,
    };

    const fixture = useDelayedFixture({
      topics,
      frame: {
        "/tf": [tf1, tf2],
        [DEFAULT_PLANNING_SCENE_TOPIC]: [planningSceneMessage],
      },
      capabilities: ["callServices"],
      activeData: {
        currentTime: { sec: 10, nsec: 0 },
      },
      callService: mockServiceClient,
    });

    return (
      <PanelSetup fixture={fixture}>
        <ThreeDeePanel
          overrideConfig={{
            ...ThreeDeePanel.defaultConfig,
            followTf: "base_link",
            layers: {
              grid: { layerId: "foxglove.Grid" },
              planningScene: {
                layerId: "foxglove.PlanningScene",
                topic: DEFAULT_PLANNING_SCENE_TOPIC,
                defaultColor: "#ffffff",
                sceneOpacity: 1.0,
                showCollisionObjects: true,
                showAttachedObjects: true,
                showOctomap: false,
              },
            },
            cameraState: {
              distance: 5.5,
              perspective: true,
              phi: rad2deg(0.5),
              targetOffset: [-0.5, 0.75, 0],
              thetaOffset: rad2deg(-0.25),
              fovy: rad2deg(0.75),
              near: 0.01,
              far: 5000,
              target: [0, 0, 0],
              targetOrientation: [0, 0, 0, 1],
            },
            topics: {
              [DEFAULT_PLANNING_SCENE_TOPIC]: { visible: true },
            },
          }}
        />
      </PanelSetup>
    );
  },
};
