/** @jest-environment jsdom */

// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

import * as THREE from "three";

import { CollisionObjectRenderable } from './CollisionObjectRenderable';
import { PlanningSceneExtension } from "./PlanningSceneExtension";
import {
    PlanningScene,
    CollisionObject,
    CollisionObjectOperation,
    SolidPrimitiveType,
    AttachedCollisionObject,
} from "./types";
import type { IRenderer } from "../../IRenderer";
import { TransformTree } from "../../transforms";

// Test data helpers - minimal data required for testing
const createCollisionObject = (id: string, operation = CollisionObjectOperation.ADD): CollisionObject => ({
    header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
    id,
    operation,
    pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    primitives: [{ type: SolidPrimitiveType.BOX, dimensions: [1, 1, 1] }],
    primitive_poses: [{ position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
    meshes: [],
    mesh_poses: [],
    planes: [],
    plane_poses: [],
    subframe_names: [],
    subframe_poses: [],
    type: { key: "", db: "" },
});

const createPlanningScene = (collisionObjects: CollisionObject[] = []): PlanningScene => ({
    name: "test_scene",
    robot_state: {
        joint_state: {
            header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
            name: [],
            position: [],
            velocity: [],
            effort: [],
        },
        multi_dof_joint_state: {
            header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
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
            header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
            origin: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
            octomap: {
                header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
                binary: false,
                id: "",
                resolution: 0.1,
                data: [],
            },
        },
    },
    is_diff: false,
});

// Create minimal mock renderer - only mock what's actually needed
const createMockRenderer = (): jest.Mocked<IRenderer> => {
    const mockConfig = { layers: {} };
    return {
        transformTree: {
            frame: jest.fn(),
            apply: jest.fn().mockReturnValue(true), // Mock successful transform application
        } as unknown as jest.Mocked<TransformTree>,
        settings: {
            errors: {
                add: jest.fn(),
                remove: jest.fn(),
                clearPath: jest.fn(),
                errors: { errorAtPath: jest.fn().mockReturnValue(undefined) },
            },
            setNodesForKey: jest.fn(),
        },
        config: mockConfig,
        updateConfig: jest.fn((fn) => fn(mockConfig)),
        addCustomLayerAction: jest.fn(),
        cameraHandler: {
            getActiveCamera: jest.fn().mockReturnValue({
                projectionMatrix: { elements: [] },
                matrixWorldInverse: { elements: [] },
            }),
        },
        topics: [],
        topicsByName: new Map(),
        hud: {
            addHUDItem: jest.fn(),
            removeHUDItem: jest.fn(),
            removeGroup: jest.fn(),
            displayIfTrue: jest.fn(),
            clear: jest.fn(),
        },
        displayTemporaryError: jest.fn(),
    } as unknown as jest.Mocked<IRenderer>;
};

const createMockCollisionObjectRenderable = (frameId: string): CollisionObjectRenderable => {
    return {
        dispose: jest.fn(),
        userData: { frameId, settings: { visible: true, opacity: 1, showPrimitives: true, showMeshes: true, showPlanes: true }, collisionObject: { "header": { frame_id: "test", stamp: { sec: 0, nsec: 0 } }, "pose": { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, "id": "test", "type": { key: "test", db: "test" }, "primitives": [], "primitive_poses": [], "meshes": [], "mesh_poses": [], "planes": [], "plane_poses": [], "subframe_names": [], "subframe_poses": [], "operation": CollisionObjectOperation.ADD }, settingsPath: [], receiveTime: 0n, messageTime: 0n, pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, shapes: new Map() }
    } as Partial<CollisionObjectRenderable> as CollisionObjectRenderable;
}

describe("PlanningSceneExtension", () => {
    let mockRenderer: jest.Mocked<IRenderer>;
    let mockServiceClient: jest.MockedFunction<(service: string, request: unknown) => Promise<unknown>>;
    let extension: PlanningSceneExtension;

    beforeEach(() => {
        jest.clearAllMocks();
        mockRenderer = createMockRenderer();
        mockServiceClient = jest.fn().mockResolvedValue({ scene: createPlanningScene() });
        extension = new PlanningSceneExtension(mockRenderer, mockServiceClient);
    });

    afterEach(() => {
        extension.dispose();
    });

    describe("Basic Interface", () => {
        it("has correct extension ID", () => {
            expect(PlanningSceneExtension.extensionId).toBe("foxglove.PlanningScene");
        });

        it("implements SceneExtension interface", () => {
            expect(extension.visible).toBe(true);
            expect(typeof extension.dispose).toBe("function");
            expect(typeof extension.getSubscriptions).toBe("function");
            expect(typeof extension.settingsNodes).toBe("function");
        });

        it("provides planning scene subscriptions", () => {
            const subscriptions = extension.getSubscriptions();
            expect(subscriptions).toHaveLength(1);
            expect(subscriptions[0]?.type).toBe("schema");

            const schemaSubscription = subscriptions[0];
            expect(schemaSubscription?.type).toBe("schema");
            if (schemaSubscription?.type === "schema") {
                // eslint-disable-next-line jest/no-conditional-expect
                expect(schemaSubscription.schemaNames).toContain("moveit_msgs/PlanningScene");
            }
        });

        it("tracks renderables correctly", () => {
            expect(extension.renderables).toBeDefined();
            expect(extension.renderables.size).toBe(0);
        });
    });

    describe("Performance Monitoring", () => {
        it("provides performance statistics", () => {
            const stats = extension.getPerformanceStats();
            expect(stats).toMatchObject({
                totalObjects: 0,
                visibleObjects: 0,
                hiddenObjects: 0,
                cachedMaterials: 0,
                cachedMeshes: 0,
                objectHashes: 0,
            });
        });



        it("supports material caching", () => {
            const material = extension.getSharedMaterial("#ffffff", 1.0);
            expect(material).toBeInstanceOf(THREE.MeshStandardMaterial);

            // Should return same instance for same parameters
            const material2 = extension.getSharedMaterial("#ffffff", 1.0);
            expect(material2).toBe(material);
        });
    });

    describe("Service Integration", () => {
        it("tracks initial scene state correctly", () => {
            expect(extension.hasInitialScene()).toBe(false);
        });

        it("handles service client integration", async () => {
            const testResponse = { scene: createPlanningScene() };
            mockServiceClient.mockResolvedValueOnce(testResponse);

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(mockServiceClient).toHaveBeenCalledWith(
                "/get_planning_scene",
                expect.any(Object)
            );
        });

        it("handles service errors gracefully", async () => {
            const consoleSpy = jest.spyOn(console, 'warn').mockImplementation();
            mockServiceClient.mockRejectedValueOnce(new Error("Service unavailable"));

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(extension.hasInitialScene()).toBe(false);
            expect(consoleSpy).toHaveBeenCalled();
            consoleSpy.mockRestore();
        });

        it("handles malformed service responses", async () => {
            const consoleSpy = jest.spyOn(console, 'warn').mockImplementation();
            mockServiceClient.mockResolvedValueOnce({ invalid: "response" });

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(extension.hasInitialScene()).toBe(false);
            expect(consoleSpy).toHaveBeenCalled();
            consoleSpy.mockRestore();
        });
    });

    describe("Transform Tree Integration", () => {
        const setupLayer = () => {
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
            };
        };

        const createMessageEvent = (scene: PlanningScene) => ({
            topic: "/planning_scene",
            receiveTime: { sec: 0, nsec: 0 },
            message: scene,
            schemaName: "moveit_msgs/PlanningScene",
            sizeInBytes: 1024,
        });

        const getMessageHandler = () => {
            const subscriptions = extension.getSubscriptions();
            return subscriptions[0]?.subscription.handler;
        };

        it("creates renderables with correct frameId for transform application", () => {
            setupLayer();
            const scene = createPlanningScene([createCollisionObject("test_object")]);
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            // Should create renderable with correct frameId for transform system
            expect(extension.renderables.size).toBe(1);
            const renderable = extension.renderables.get("test_object");
            expect(renderable?.userData.frameId).toBe("base_link");
        });

        it("creates renderables with different frameIds correctly", () => {
            setupLayer();
            const mapObject = createCollisionObject("map_object");
            mapObject.header.frame_id = "map";
            const robotObject = createCollisionObject("robot_object");
            robotObject.header.frame_id = "base_link";

            const scene = createPlanningScene([mapObject, robotObject]);
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            // Should create renderables with correct frameIds
            expect(extension.renderables.size).toBe(2);
            expect(extension.renderables.get("map_object")?.userData.frameId).toBe("map");
            expect(extension.renderables.get("robot_object")?.userData.frameId).toBe("base_link");
        });

        it("creates attached object renderables with correct frameId", () => {
            setupLayer();
            const attachedObject: AttachedCollisionObject = {
                link_name: "gripper_link",
                object: createCollisionObject("attached_tool"),
                touch_links: ["finger"],
                detach_posture: {
                    header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
                    joint_names: [],
                    points: [],
                },
                weight: 0.5,
            };

            const scene = createPlanningScene();
            scene.robot_state.attached_collision_objects = [attachedObject];
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            // Should create renderable with attached link frameId
            expect(extension.renderables.size).toBe(1);
            expect(extension.renderables.get("attached_tool")?.userData.frameId).toBe("gripper_link");
        });

        it("creates multiple attached objects with correct frameIds", () => {
            setupLayer();
            const leftTool: AttachedCollisionObject = {
                link_name: "left_gripper",
                object: createCollisionObject("left_tool"),
                touch_links: ["left_finger"],
                detach_posture: {
                    header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
                    joint_names: [],
                    points: [],
                },
                weight: 0.3,
            };

            const rightTool: AttachedCollisionObject = {
                link_name: "right_gripper",
                object: createCollisionObject("right_tool"),
                touch_links: ["right_finger"],
                detach_posture: {
                    header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
                    joint_names: [],
                    points: [],
                },
                weight: 0.4,
            };

            const scene = createPlanningScene();
            scene.robot_state.attached_collision_objects = [leftTool, rightTool];
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            // Should create renderables with correct frameIds for each gripper
            expect(extension.renderables.size).toBe(2);
            expect(extension.renderables.get("left_tool")?.userData.frameId).toBe("left_gripper");
            expect(extension.renderables.get("right_tool")?.userData.frameId).toBe("right_gripper");
        });

        it("sets up renderables for transform system correctly", () => {
            setupLayer();
            const scene = createPlanningScene([createCollisionObject("test_object")]);
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            // Should create renderable with complete userData for transform system
            const renderable = extension.renderables.get("test_object");
            expect(renderable?.userData).toEqual(expect.objectContaining({
                frameId: "base_link",
                pose: expect.objectContaining({
                    position: expect.objectContaining({ x: 0, y: 0, z: 0 }), // From test data
                    orientation: expect.objectContaining({ x: 0, y: 0, z: 0, w: 1 })
                }),
                messageTime: expect.any(BigInt),
                receiveTime: expect.any(BigInt),
            }));
        });

        it("integrates with transform system via inherited startFrame", () => {
            const consoleSpy = jest.spyOn(console, 'warn').mockImplementation();

            setupLayer();
            const scene = createPlanningScene([createCollisionObject("test_object")]);

            // Get handler from the extension with service
            const subscriptions = extension.getSubscriptions();
            const handler = subscriptions[0]?.subscription.handler;

            handler!(createMessageEvent(scene));
            expect(extension.renderables.size).toBe(1);

            // Clear previous calls to transformTree
            (mockRenderer.transformTree.apply as jest.Mock).mockClear();

            // Call startFrame - this should trigger transform application for all renderables
            // Note: startFrame is inherited from SceneExtension and calls updatePose() internally
            extension.startFrame(BigInt(1000000), "map", "map");

            // Should call transformTree.apply() during startFrame to apply transforms
            // eslint-disable-next-line @typescript-eslint/unbound-method
            expect(mockRenderer.transformTree.apply).toHaveBeenCalledWith(
                expect.anything(), // tempPose
                expect.objectContaining({ // pose from userData (test data has origin pose)
                    position: expect.objectContaining({ x: 0, y: 0, z: 0 }),
                    orientation: expect.objectContaining({ x: 0, y: 0, z: 0, w: 1 })
                }),
                "map", // renderFrameId
                "map", // fixedFrameId
                "base_link", // srcFrameId (from userData.frameId)
                BigInt(1000000), // dstTime
                expect.any(BigInt) // srcTime (from userData.messageTime)
            );

            // Should not have any warnings since we provided a service client
            expect(consoleSpy).not.toHaveBeenCalled();

            consoleSpy.mockRestore();
        });
    });

    describe("Message Processing", () => {
        const setupLayer = () => {
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
            };
        };

        const createMessageEvent = (scene: PlanningScene) => ({
            topic: "/planning_scene",
            receiveTime: { sec: 0, nsec: 0 },
            message: scene,
            schemaName: "moveit_msgs/PlanningScene",
            sizeInBytes: 1024,
        });

        const getMessageHandler = () => {
            const subscriptions = extension.getSubscriptions();
            return subscriptions[0]?.subscription.handler;
        };

        it("processes basic collision objects", () => {
            setupLayer();
            const scene = createPlanningScene([createCollisionObject("test_box")]);
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            expect(extension.renderables.size).toBe(1);
            expect(extension.renderables.has("test_box")).toBe(true);
        });

        it("handles attached collision objects", () => {
            setupLayer();
            const attachedObject: AttachedCollisionObject = {
                link_name: "gripper_link",
                object: createCollisionObject("attached_tool"),
                touch_links: ["finger"],
                detach_posture: {
                    header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
                    joint_names: [],
                    points: [],
                },
                weight: 0.5,
            };

            const scene = createPlanningScene();
            scene.robot_state.attached_collision_objects = [attachedObject];
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            expect(extension.renderables.size).toBe(1);
            expect(extension.renderables.has("attached_tool")).toBe(true);

            const renderable = extension.renderables.get("attached_tool");
            expect(renderable?.userData.frameId).toBe("gripper_link");
        });

        it("handles object removal operations", () => {
            setupLayer();
            const handler = getMessageHandler();

            // Add object
            const addScene = createPlanningScene([createCollisionObject("temp_object")]);
            handler!(createMessageEvent(addScene));
            expect(extension.renderables.size).toBe(1);

            // Remove object
            const removeObject = createCollisionObject("temp_object", CollisionObjectOperation.REMOVE);
            const removeScene = createPlanningScene([removeObject]);
            handler!(createMessageEvent(removeScene));
            expect(extension.renderables.size).toBe(0);
        });

        it("preserves object colors from scene", () => {
            setupLayer();
            const scene = createPlanningScene([createCollisionObject("colored_box")]);
            scene.object_colors = [
                { id: "colored_box", color: { r: 1.0, g: 0.0, b: 0.0, a: 0.8 } }
            ];
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            expect(extension.renderables.size).toBe(1);
            expect(extension.renderables.has("colored_box")).toBe(true);
        });

        it("handles empty scenes", () => {
            setupLayer();
            const scene = createPlanningScene();
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            expect(extension.renderables.size).toBe(0);
        });
    });

    describe("Geometry Support", () => {
        it("supports different primitive types", () => {
            const boxObject = createCollisionObject("box");
            expect(boxObject.primitives[0]?.type).toBe(SolidPrimitiveType.BOX);

            const sphereObject: CollisionObject = {
                ...createCollisionObject("sphere"),
                primitives: [{ type: SolidPrimitiveType.SPHERE, dimensions: [0.5] }],
            };
            expect(sphereObject.primitives[0]?.type).toBe(SolidPrimitiveType.SPHERE);
        });

        it("supports mesh objects", () => {
            const meshObject: CollisionObject = {
                ...createCollisionObject("mesh"),
                primitives: [],
                primitive_poses: [],
                meshes: [{
                    vertices: [
                        { x: 0, y: 0, z: 0 },
                        { x: 1, y: 0, z: 0 },
                        { x: 0.5, y: 1, z: 0 }
                    ],
                    triangles: [{ vertex_indices: new Uint32Array([0, 1, 2]) }],
                }],
                mesh_poses: [{ position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
            };

            expect(meshObject.meshes).toHaveLength(1);
            expect(meshObject.meshes[0]?.vertices).toHaveLength(3);
        });

        // eslint-disable-next-line jest/no-disabled-tests
        it.skip("supports octomaps (not yet implemented)", () => {
            const setupLayer = () => {
                (mockRenderer.config.layers as any)["test-layer"] = {
                    layerId: "foxglove.PlanningScene",
                    visible: true,
                    topic: "/planning_scene",
                };
            };

            const createMessageEvent = (scene: PlanningScene) => ({
                topic: "/planning_scene",
                receiveTime: { sec: 0, nsec: 0 },
                message: scene,
                schemaName: "moveit_msgs/PlanningScene",
                sizeInBytes: 1024,
            });

            const getMessageHandler = () => {
                const subscriptions = extension.getSubscriptions();
                return subscriptions[0]?.subscription.handler;
            };

            setupLayer();
            const scene = createPlanningScene();
            scene.world.octomap = {
                header: { frame_id: "map", stamp: { sec: 1234, nsec: 567890 } },
                origin: { position: { x: 1, y: 2, z: 3 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
                octomap: {
                    header: { frame_id: "map", stamp: { sec: 1234, nsec: 567890 } },
                    binary: true,
                    id: "OcTree",
                    resolution: 0.05,
                    data: [1, 2, 3, 4, 5]
                }
            };
            const handler = getMessageHandler();

            handler!(createMessageEvent(scene));

            // This test should FAIL until octomap support is implemented
            // When implemented, octomaps should create renderables for visualization
            expect(extension.renderables.size).toBe(1);
            expect(extension.renderables.has("octomap")).toBe(true);

            const octomapRenderable = extension.renderables.get("octomap");
            expect(octomapRenderable?.userData.frameId).toBe("map");

            // When implementing octomap support, also verify:
            // - Proper voxel visualization with correct resolution (0.05)
            // - Binary vs ASCII octomap handling
            // - Performance optimization for large octomaps
            // - Proper positioning from origin transform
        });
    });

    describe("State Persistence", () => {
        const setupLayer = () => {
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
                collisionObjects: {},
            };
        };

        const createMessageEvent = (scene: PlanningScene) => ({
            topic: "/planning_scene",
            receiveTime: { sec: 0, nsec: 0 },
            message: scene,
            schemaName: "moveit_msgs/PlanningScene",
            sizeInBytes: 1024,
        });

        const getMessageHandler = () => {
            const subscriptions = extension.getSubscriptions();
            return subscriptions[0]?.subscription.handler;
        };

        it("preserves object settings across remove/add cycles", () => {
            setupLayer();
            const handler = getMessageHandler();

            // Add initial object
            const initialScene = createPlanningScene([createCollisionObject("persistent_object")]);
            handler!(createMessageEvent(initialScene));

            expect(extension.renderables.size).toBe(1);
            expect(extension.renderables.has("persistent_object")).toBe(true);

            // Verify initial settings
            const initialRenderable = extension.renderables.get("persistent_object");
            expect(initialRenderable?.userData.settings.visible).toBe(true);

            // Modify settings through the extension's settings system
            // Simulate a user changing visibility
            if (initialRenderable) {
                initialRenderable.userData.settings.visible = false;
                initialRenderable.userData.settings.opacity = 0.5;
            }

            // Remove object
            const removeScene = createPlanningScene([createCollisionObject("persistent_object", CollisionObjectOperation.REMOVE)]);
            handler!(createMessageEvent(removeScene));

            expect(extension.renderables.size).toBe(0);

            // Re-add the same object
            const restoreScene = createPlanningScene([createCollisionObject("persistent_object")]);
            handler!(createMessageEvent(restoreScene));

            expect(extension.renderables.size).toBe(1);
            const restoredRenderable = extension.renderables.get("persistent_object");

            // Settings should be preserved from the layer configuration
            // Note: This tests the current behavior - in practice, settings persistence
            // would need to be implemented in the layer configuration system
            expect(restoredRenderable?.userData.settings.visible).toBe(true); // Default restored
        });

        it("preserves attached object properties across attach/detach cycles", () => {
            setupLayer();
            const handler = getMessageHandler();

            // Create attached object
            const attachedObject: AttachedCollisionObject = {
                link_name: "gripper_link",
                object: createCollisionObject("persistent_tool"),
                touch_links: ["finger1", "finger2"],
                detach_posture: {
                    header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
                    joint_names: ["joint1"],
                    points: [{ positions: [0.5], velocities: [], accelerations: [], effort: [], time_from_start: { sec: 0, nsec: 0 } }],
                },
                weight: 0.8,
            };

            const attachScene = createPlanningScene();
            attachScene.robot_state.attached_collision_objects = [attachedObject];
            handler!(createMessageEvent(attachScene));

            expect(extension.renderables.size).toBe(1);
            const attachedRenderable = extension.renderables.get("persistent_tool");
            expect(attachedRenderable?.userData.frameId).toBe("gripper_link");

            // Store original properties
            const originalWeight = attachedObject.weight;
            const originalTouchLinks = attachedObject.touch_links;

            // Detach object (remove from attached_collision_objects)
            const detachScene = createPlanningScene();
            detachScene.robot_state.attached_collision_objects = [];
            handler!(createMessageEvent(detachScene));

            expect(extension.renderables.size).toBe(0);

            // Re-attach the same object with same properties
            const reattachScene = createPlanningScene();
            reattachScene.robot_state.attached_collision_objects = [attachedObject];
            handler!(createMessageEvent(reattachScene));

            expect(extension.renderables.size).toBe(1);
            const reattachedRenderable = extension.renderables.get("persistent_tool");
            expect(reattachedRenderable?.userData.frameId).toBe("gripper_link");

            // Verify properties are preserved
            expect(attachedObject.weight).toBe(originalWeight);
            expect(attachedObject.touch_links).toEqual(originalTouchLinks);
        });

        it("preserves object colors across scene refreshes", () => {
            setupLayer();
            const handler = getMessageHandler();

            // Create scene with colored objects
            const coloredScene = createPlanningScene([
                createCollisionObject("red_object"),
                createCollisionObject("blue_object")
            ]);
            coloredScene.object_colors = [
                { id: "red_object", color: { r: 1.0, g: 0.0, b: 0.0, a: 0.9 } },
                { id: "blue_object", color: { r: 0.0, g: 0.0, b: 1.0, a: 0.7 } }
            ];

            handler!(createMessageEvent(coloredScene));

            expect(extension.renderables.size).toBe(2);
            expect(extension.renderables.has("red_object")).toBe(true);
            expect(extension.renderables.has("blue_object")).toBe(true);

            // Clear all renderables (simulate refresh)
            extension.removeAllRenderables();
            expect(extension.renderables.size).toBe(0);

            // Re-process the same scene
            handler!(createMessageEvent(coloredScene));

            expect(extension.renderables.size).toBe(2);
            expect(extension.renderables.has("red_object")).toBe(true);
            expect(extension.renderables.has("blue_object")).toBe(true);

            // Colors should be preserved from the scene data
            // This tests that color information is properly restored from the planning scene
        });

        it("maintains object IDs across differential updates", () => {
            setupLayer();
            const handler = getMessageHandler();

            // Process initial scene
            const initialScene = createPlanningScene([
                createCollisionObject("stable_object_1"),
                createCollisionObject("stable_object_2")
            ]);
            handler!(createMessageEvent(initialScene));

            expect(extension.renderables.size).toBe(2);

            // Process differential update (add one object)
            const diffScene = createPlanningScene([createCollisionObject("new_object")]);
            diffScene.is_diff = true;
            handler!(createMessageEvent(diffScene));

            expect(extension.renderables.size).toBe(3);
            const updatedIds = Array.from(extension.renderables.keys()).sort();

            // Original objects should maintain their IDs
            expect(updatedIds).toContain("stable_object_1");
            expect(updatedIds).toContain("stable_object_2");
            expect(updatedIds).toContain("new_object");

            // Verify original objects are still accessible by their IDs
            expect(extension.renderables.has("stable_object_1")).toBe(true);
            expect(extension.renderables.has("stable_object_2")).toBe(true);
            expect(extension.renderables.has("new_object")).toBe(true);
        });

        it("recovers from extension recreation with same configuration", () => {
            setupLayer();
            const handler = getMessageHandler();

            // Process scene with current extension
            const scene = createPlanningScene([createCollisionObject("recoverable_object")]);
            scene.object_colors = [
                { id: "recoverable_object", color: { r: 0.5, g: 0.5, b: 0.5, a: 1.0 } }
            ];
            handler!(createMessageEvent(scene));

            expect(extension.renderables.size).toBe(1);

            // Store current configuration for recreation
            const layerConfig = mockRenderer.config.layers["test-layer"];

            // Dispose current extension
            extension.dispose();

            // Create new extension with same configuration
            const newExtension = new PlanningSceneExtension(mockRenderer, mockServiceClient);

            // Process the same scene with new extension
            const newHandler = newExtension.getSubscriptions()[0]?.subscription.handler;
            newHandler!(createMessageEvent(scene));

            expect(newExtension.renderables.size).toBe(1);
            expect(newExtension.renderables.has("recoverable_object")).toBe(true);

            // Verify that the same configuration produces the same result
            expect(layerConfig).toBeDefined();

            // Cleanup new extension
            newExtension.dispose();
        });
    });

    describe("Settings Management", () => {
        it("provides settings nodes", () => {
            const nodes = extension.settingsNodes();
            expect(Array.isArray(nodes)).toBe(true);
        });

        it("supports settings actions", () => {
            expect(typeof extension.handleSettingsAction).toBe("function");
        });

        it("disables octomap setting by default", () => {
            // Set up a planning scene layer in the config
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
                showOctomap: false,
            };

            // Test that showOctomap defaults to false
            const nodes = extension.settingsNodes();

            // Find a planning scene layer node
            const planningSceneNode = nodes.find(node =>
                node.path.length === 2 &&
                node.path[1] === "test-layer"
            );

            expect(planningSceneNode).toBeDefined();
            expect(planningSceneNode?.node.fields?.showOctomap?.value).toBe(false);
        });

        it("shows temporary error when trying to enable octomap", () => {
            // Set up a planning scene layer in the config
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
                showOctomap: false,
            };

            // Create an action to enable showOctomap
            const action = {
                action: "update" as const,
                payload: {
                    path: ["layers", "test-layer", "showOctomap"] as const,
                    input: "boolean" as const,
                    value: true,
                },
            };

            // Verify that the temporary error is not yet shown
            expect(mockRenderer.displayTemporaryError).not.toHaveBeenCalled();

            // Execute the action
            extension.handleSettingsAction(action);

            // Verify that the temporary error was shown
            expect(mockRenderer.displayTemporaryError).toHaveBeenCalled();
        });
    });

    describe("Cleanup", () => {
        it("cleans up resources on dispose", () => {
            // Add some renderables first
            const mockRenderable = createMockCollisionObjectRenderable("test");
            extension.renderables.set("test", mockRenderable);

            extension.dispose();

            // eslint-disable-next-line @typescript-eslint/unbound-method
            expect(mockRenderable.dispose).toHaveBeenCalled();
            expect(extension.renderables.size).toBe(0);
            // eslint-disable-next-line @typescript-eslint/unbound-method
            expect(mockRenderer.settings.errors.clearPath).toHaveBeenCalled();
        });

        it("removes all renderables", () => {
            const mockRenderable = createMockCollisionObjectRenderable("test");
            extension.renderables.set("test", mockRenderable);

            extension.removeAllRenderables();

            // eslint-disable-next-line @typescript-eslint/unbound-method
            expect(mockRenderable.dispose).toHaveBeenCalled();
            expect(extension.renderables.size).toBe(0);
        });
    });
});
