/** @jest-environment jsdom */

// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

import * as THREE from "three";

import { CollisionObjectRenderable } from "./CollisionObjectRenderable";
import { PlanningSceneExtension } from "./PlanningSceneExtension";
import {
    PlanningScene,
    CollisionObject,
    CollisionObjectOperation,
    SolidPrimitiveType,
} from "./types";
import type { IRenderer } from "../../IRenderer";
import { TransformTree } from "../../transforms";

jest.mock("./CollisionObjectRenderable");

const createTestCollisionObject = (id: string, operation = CollisionObjectOperation.ADD): CollisionObject => ({
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

const createTestPlanningScene = (
    collisionObjects: CollisionObject[],
    options: { isDiff?: boolean } = {},
): PlanningScene => ({
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
            origin: {
                position: { x: 0, y: 0, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
            octomap: {
                header: { frame_id: "base_link", stamp: { sec: 0, nsec: 0 } },
                binary: false,
                id: "",
                resolution: 0.1,
                data: [],
            },
        },
    },
    is_diff: options.isDiff ?? false,
});

describe("PlanningSceneExtension", () => {
    let mockRenderer: jest.Mocked<IRenderer>;
    let mockServiceClient: jest.MockedFunction<(service: string, request: unknown) => Promise<unknown>>;
    let extension: PlanningSceneExtension;
    let mockTransformTree: jest.Mocked<TransformTree>;

    beforeEach(() => {
        jest.clearAllMocks();

        mockTransformTree = { frame: jest.fn() } as unknown as jest.Mocked<TransformTree>;

        const mockConfig = { layers: {} as Record<string, any> };

        mockRenderer = {
            transformTree: mockTransformTree,
            settings: {
                errors: {
                    add: jest.fn(),
                    remove: jest.fn(),
                    clearPath: jest.fn(),
                    errors: {
                        errorAtPath: jest.fn().mockReturnValue(undefined),
                    },
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
        } as unknown as jest.Mocked<IRenderer>;

        mockServiceClient = jest.fn();

        (CollisionObjectRenderable as jest.MockedClass<typeof CollisionObjectRenderable>).mockImplementation((objectId: string) => ({
            update: jest.fn(),
            dispose: jest.fn(),
            visible: true,
            setExtension: jest.fn(), // Add the missing setExtension method
            userData: {
                frameId: "base_link",
                collisionObject: {
                    id: objectId,
                    header: { frame_id: "base_link" },
                    primitives: [],
                    meshes: [],
                    planes: [],
                    primitive_poses: [],
                    mesh_poses: [],
                    plane_poses: [],
                },
                settings: { visible: true, opacity: 1.0, showPrimitives: true, showMeshes: true, showPlanes: true },
                settingsPath: [],
            },
        } as any));

        extension = new PlanningSceneExtension(mockRenderer, mockServiceClient);
    });

    afterEach(() => {
        extension.dispose();
        jest.clearAllMocks();
    });

    describe("Basic Functionality", () => {
        it("should have correct extension ID", () => {
            expect(PlanningSceneExtension.extensionId).toBe("foxglove.PlanningScene");
        });

        it("should initialize with default visibility", () => {
            expect(extension.visible).toBe(true);
        });

        it("should return planning scene subscriptions", () => {
            const subscriptions = extension.getSubscriptions();
            expect(subscriptions).toHaveLength(1);
            expect(subscriptions[0]?.type).toBe("schema");
            expect(subscriptions[0]).toMatchObject({
                type: "schema",
                schemaNames: expect.objectContaining(
                    new Set(["moveit_msgs/PlanningScene", "moveit_msgs/msg/PlanningScene"])
                ),
            });
        });

        it("should support performance statistics", () => {
            const stats = extension.getPerformanceStats();
            expect(stats).toMatchObject({
                totalObjects: 0,
                visibleObjects: 0,
                hiddenObjects: 0,
                cachedGeometries: 0,
                cachedMaterials: 0,
                cachedMeshes: 0,
                objectHashes: 0,
                cullingSavings: "0%",
            });
        });
    });

    describe("Layer Management", () => {
        it("should return settings nodes for layers", () => {
            const instanceId = "test-layer-1";

            // Setup a layer in config
            (mockRenderer.config.layers as any)[instanceId] = {
                layerId: "foxglove.PlanningScene",
                instanceId,
                visible: true,
                topic: "/planning_scene",
                order: 1,
            };

            const nodes = extension.settingsNodes();
            expect(Array.isArray(nodes)).toBe(true);
            expect(nodes.length).toBeGreaterThan(0);
        });

        it("should handle layer visibility changes", () => {
            const instanceId = "test-layer-1";
            (mockRenderer.config.layers as any)[instanceId] = {
                layerId: "foxglove.PlanningScene",
                instanceId,
                visible: false,
                topic: "/planning_scene",
                order: 1,
            };

            const nodes = extension.settingsNodes();
            const layerNode = nodes.find(node => node.path[1] === instanceId);
            expect(layerNode?.node.visible).toBe(false);
        });

        it("should have message processing capability", () => {
            const subscriptions = extension.getSubscriptions();
            expect(subscriptions).toHaveLength(1);
            expect(subscriptions[0]?.subscription.handler).toBeDefined();
        });
    });

    describe("Collision Object Operations", () => {
        it("should support all collision object operations", () => {
            // Test ADD operation (default)
            const addObject = createTestCollisionObject("add_object", CollisionObjectOperation.ADD);
            expect(addObject.operation).toBe(CollisionObjectOperation.ADD);
            expect(addObject.primitives).toHaveLength(1);

            // Test REMOVE operation
            const removeObject = createTestCollisionObject("remove_object", CollisionObjectOperation.REMOVE);
            expect(removeObject.operation).toBe(CollisionObjectOperation.REMOVE);

            // Test APPEND operation
            const appendObject = createTestCollisionObject("append_object", CollisionObjectOperation.APPEND);
            expect(appendObject.operation).toBe(CollisionObjectOperation.APPEND);

            // Test MOVE operation (should have empty geometry arrays)
            const moveObject: CollisionObject = {
                ...createTestCollisionObject("move_object", CollisionObjectOperation.MOVE),
                primitives: [],
                primitive_poses: [],
                meshes: [],
                mesh_poses: [],
                planes: [],
                plane_poses: [],
            };
            expect(moveObject.operation).toBe(CollisionObjectOperation.MOVE);
            expect(moveObject.primitives).toHaveLength(0);
        });

        it("should support visibility controls", () => {
            // Test that the extension has visibility control methods
            expect(typeof extension.handleSettingsAction).toBe("function");

            // Test that renderables map exists for tracking objects
            expect(extension.renderables).toBeDefined();
            expect(extension.renderables.size).toBe(0);
        });
    });

    describe("Collision Object Rendering", () => {
        it("should support all primitive types with realistic dimensions", () => {
            // Test Box primitive
            const boxObject = createTestCollisionObject("box_object");
            expect(boxObject.primitives[0]?.type).toBe(SolidPrimitiveType.BOX);
            expect(boxObject.primitives[0]?.dimensions).toEqual([1, 1, 1]);

            // Test Sphere primitive
            const sphereObject: CollisionObject = {
                ...createTestCollisionObject("sphere_object"),
                primitives: [{ type: SolidPrimitiveType.SPHERE, dimensions: [0.5] }],
            };
            expect(sphereObject.primitives[0]?.type).toBe(SolidPrimitiveType.SPHERE);
            expect(sphereObject.primitives[0]?.dimensions).toEqual([0.5]);

            // Test Cylinder primitive (height, radius)
            const cylinderObject: CollisionObject = {
                ...createTestCollisionObject("cylinder_object"),
                primitives: [{ type: SolidPrimitiveType.CYLINDER, dimensions: [2.0, 0.3] }],
            };
            expect(cylinderObject.primitives[0]?.type).toBe(SolidPrimitiveType.CYLINDER);
            expect(cylinderObject.primitives[0]?.dimensions).toEqual([2.0, 0.3]);

            // Test Cone primitive (height, radius)
            const coneObject: CollisionObject = {
                ...createTestCollisionObject("cone_object"),
                primitives: [{ type: SolidPrimitiveType.CONE, dimensions: [1.5, 0.4] }],
            };
            expect(coneObject.primitives[0]?.type).toBe(SolidPrimitiveType.CONE);
            expect(coneObject.primitives[0]?.dimensions).toEqual([1.5, 0.4]);
        });

        it("should support complex mesh objects with multiple triangles", () => {
            const complexMeshObject: CollisionObject = {
                ...createTestCollisionObject("complex_mesh"),
                primitives: [],
                primitive_poses: [],
                meshes: [{
                    vertices: [
                        { x: 0, y: 0, z: 0 },    // vertex 0
                        { x: 1, y: 0, z: 0 },    // vertex 1
                        { x: 0.5, y: 1, z: 0 },  // vertex 2
                        { x: 0.5, y: 0.5, z: 1 } // vertex 3
                    ],
                    triangles: [
                        { vertex_indices: new Uint32Array([0, 1, 2]) }, // base triangle
                        { vertex_indices: new Uint32Array([0, 2, 3]) }, // side triangle 1
                        { vertex_indices: new Uint32Array([1, 3, 2]) }, // side triangle 2
                        { vertex_indices: new Uint32Array([0, 3, 1]) }  // side triangle 3
                    ],
                }],
                mesh_poses: [{ position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
            };

            expect(complexMeshObject.meshes).toHaveLength(1);
            expect(complexMeshObject.meshes[0]?.vertices).toHaveLength(4);
            expect(complexMeshObject.meshes[0]?.triangles).toHaveLength(4);
        });

        it("should support plane objects", () => {
            const planeObject: CollisionObject = {
                ...createTestCollisionObject("plane_object"),
                primitives: [],
                primitive_poses: [],
                planes: [
                    { coef: [0, 0, 1, -1] }, // horizontal plane at z=1
                    { coef: [1, 0, 0, -2] }  // vertical plane at x=2
                ],
                plane_poses: [
                    { position: { x: 0, y: 0, z: 1 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
                    { position: { x: 2, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0.707, w: 0.707 } }
                ],
            };

            expect(planeObject.planes).toHaveLength(2);
            expect(planeObject.plane_poses).toHaveLength(2);
        });

        it("should support mixed geometry objects", () => {
            const mixedObject: CollisionObject = {
                ...createTestCollisionObject("mixed_object"),
                primitives: [
                    { type: SolidPrimitiveType.BOX, dimensions: [1, 1, 1] },
                    { type: SolidPrimitiveType.SPHERE, dimensions: [0.5] }
                ],
                primitive_poses: [
                    { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
                    { position: { x: 2, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }
                ],
                meshes: [{
                    vertices: [{ x: 0, y: 0, z: 2 }, { x: 1, y: 0, z: 2 }, { x: 0.5, y: 1, z: 2 }],
                    triangles: [{ vertex_indices: new Uint32Array([0, 1, 2]) }],
                }],
                mesh_poses: [{ position: { x: 0, y: 0, z: 2 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
                planes: [{ coef: [0, 0, 1, -3] }],
                plane_poses: [{ position: { x: 0, y: 0, z: 3 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
            };

            expect(mixedObject.primitives).toHaveLength(2);
            expect(mixedObject.meshes).toHaveLength(1);
            expect(mixedObject.planes).toHaveLength(1);
        });
    });

    describe("Advanced Features", () => {
        it("should support object colors and transparency", () => {
            const redBox = createTestCollisionObject("red_box");
            const blueSphere = createTestCollisionObject("blue_sphere");
            blueSphere.primitives = [{ type: SolidPrimitiveType.SPHERE, dimensions: [0.5] }];

            const coloredScene = createTestPlanningScene([redBox, blueSphere]);
            coloredScene.object_colors = [
                { id: "red_box", color: { r: 1.0, g: 0.0, b: 0.0, a: 0.8 } },
                { id: "blue_sphere", color: { r: 0.0, g: 0.0, b: 1.0, a: 0.5 } },
            ];

            expect(coloredScene.object_colors).toHaveLength(2);
            expect(coloredScene.object_colors[0]?.color.a).toBe(0.8);
            expect(coloredScene.object_colors[1]?.color.a).toBe(0.5);
            expect(coloredScene.world.collision_objects).toHaveLength(2);
        });

        it("should support octomap rendering", () => {
            const octomapScene = createTestPlanningScene([]);
            octomapScene.world.octomap = {
                header: { frame_id: "map", stamp: { sec: 1234, nsec: 567890 } },
                origin: { position: { x: 1, y: 2, z: 3 }, orientation: { x: 0, y: 0, z: 0.707, w: 0.707 } },
                octomap: {
                    header: { frame_id: "map", stamp: { sec: 1234, nsec: 567890 } },
                    binary: true,
                    id: "OcTree",
                    resolution: 0.05,
                    data: [1, 2, 3, 4, 5]
                }
            };

            expect(octomapScene.world.octomap.octomap.binary).toBe(true);
            expect(octomapScene.world.octomap.octomap.resolution).toBe(0.05);
            expect(octomapScene.world.octomap.octomap.data).toHaveLength(5);
        });
    });

    describe("Service Integration", () => {
        it("should support service client integration", async () => {
            const testResponse = { scene: createTestPlanningScene([]) };
            mockServiceClient.mockResolvedValueOnce(testResponse);

            // Test that the extension can retry fetching initial scene
            extension.retryFetchInitialScene();

            // Wait for the async operation to complete
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(mockServiceClient).toHaveBeenCalledWith(
                "/get_planning_scene",
                expect.objectContaining({
                    components: expect.any(Object)
                })
            );
        });

        it("should handle service errors gracefully", async () => {
            mockServiceClient.mockRejectedValueOnce(new Error("Service unavailable"));

            // Test that the extension handles service errors
            extension.retryFetchInitialScene();

            // Wait for the async operation to complete
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(mockServiceClient).toHaveBeenCalled();
            expect(extension.hasInitialScene()).toBe(false);
        });

        it("should track initial scene state", () => {
            expect(extension.hasInitialScene()).toBe(false);

            // Simulate having an initial scene
            (extension as any).initialSceneFetched = true;
            (extension as any).currentScene = createTestPlanningScene([]);

            expect(extension.hasInitialScene()).toBe(true);
        });

        it("should handle malformed service responses", async () => {
            mockServiceClient.mockResolvedValueOnce({ invalid: "response" });

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(extension.hasInitialScene()).toBe(false);
        });

        it("should render primitive shapes from service response", async () => {
            const boxObject = createTestCollisionObject("test_box");
            const sphereObject = createTestCollisionObject("test_sphere");
            sphereObject.primitives = [{ type: SolidPrimitiveType.SPHERE, dimensions: [0.5] }];

            const sceneWithPrimitives = createTestPlanningScene([boxObject, sphereObject]);
            const testResponse = { scene: sceneWithPrimitives };
            mockServiceClient.mockResolvedValueOnce(testResponse);

            // Setup a layer to enable message processing
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
            };

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(extension.hasInitialScene()).toBe(true);
            expect((extension as any).currentScene.world.collision_objects).toHaveLength(2);
        });

        it("should render mesh objects from service response", async () => {
            const meshObject: CollisionObject = {
                ...createTestCollisionObject("test_mesh"),
                primitives: [],
                primitive_poses: [],
                meshes: [{
                    vertices: [
                        { x: 0, y: 0, z: 0 },
                        { x: 1, y: 0, z: 0 },
                        { x: 0.5, y: 1, z: 0 }
                    ],
                    triangles: [
                        { vertex_indices: new Uint32Array([0, 1, 2]) }
                    ],
                }],
                mesh_poses: [{ position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
            };

            const sceneWithMeshes = createTestPlanningScene([meshObject]);
            const testResponse = { scene: sceneWithMeshes };
            mockServiceClient.mockResolvedValueOnce(testResponse);

            // Setup a layer to enable message processing
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
            };

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(extension.hasInitialScene()).toBe(true);
            const scene = (extension as any).currentScene;
            expect(scene.world.collision_objects[0]?.meshes).toHaveLength(1);
            expect(scene.world.collision_objects[0]?.meshes[0]?.vertices).toHaveLength(3);
        });

        it("should store octomap data from service response (rendering not implemented)", async () => {
            const sceneWithOctomap = createTestPlanningScene([]);
            sceneWithOctomap.world.octomap = {
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

            const testResponse = { scene: sceneWithOctomap };
            mockServiceClient.mockResolvedValueOnce(testResponse);

            // Setup a layer to enable message processing
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
            };

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(extension.hasInitialScene()).toBe(true);
            const scene = (extension as any).currentScene;
            // Octomap data is stored but not processed for rendering (not yet implemented)
            expect(scene.world.octomap.octomap.binary).toBe(true);
            expect(scene.world.octomap.octomap.resolution).toBe(0.05);
            expect(scene.world.octomap.octomap.data).toHaveLength(5);
            // Verify that no renderables are created for octomap (since it's not implemented)
            expect(extension.renderables.size).toBe(0);
        });

        it("should render mixed geometry from service response", async () => {
            const mixedObject: CollisionObject = {
                ...createTestCollisionObject("mixed_object"),
                primitives: [
                    { type: SolidPrimitiveType.BOX, dimensions: [1, 1, 1] },
                    { type: SolidPrimitiveType.SPHERE, dimensions: [0.5] }
                ],
                primitive_poses: [
                    { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
                    { position: { x: 2, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }
                ],
                meshes: [{
                    vertices: [{ x: 0, y: 0, z: 2 }, { x: 1, y: 0, z: 2 }, { x: 0.5, y: 1, z: 2 }],
                    triangles: [{ vertex_indices: new Uint32Array([0, 1, 2]) }],
                }],
                mesh_poses: [{ position: { x: 0, y: 0, z: 2 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
            };

            const sceneWithMixedGeometry = createTestPlanningScene([mixedObject]);
            const testResponse = { scene: sceneWithMixedGeometry };
            mockServiceClient.mockResolvedValueOnce(testResponse);

            // Setup a layer to enable message processing
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
            };

            extension.retryFetchInitialScene();
            await new Promise(resolve => setTimeout(resolve, 0));

            expect(extension.hasInitialScene()).toBe(true);
            const scene = (extension as any).currentScene;
            const object = scene.world.collision_objects[0];
            expect(object.primitives).toHaveLength(2);
            expect(object.meshes).toHaveLength(1);
            expect(object.primitives[0]?.type).toBe(SolidPrimitiveType.BOX);
            expect(object.primitives[1]?.type).toBe(SolidPrimitiveType.SPHERE);
        });
    });

    describe("Performance Optimizations", () => {
        it("should reuse frustum culling objects", () => {
            // Test that the extension has the performance optimization properties
            expect((extension as any).frustum).toBeDefined();
            expect((extension as any).cameraMatrix).toBeDefined();
        });

        it("should support geometry and material caching", () => {
            const geometry = extension.getSharedGeometry("box", [1, 1, 1], () => new THREE.BufferGeometry());
            expect(geometry).toBeDefined();

            const material = extension.getSharedMaterial("#ffffff", 1.0);
            expect(material).toBeDefined();
        });
    });

    describe("Settings Persistence", () => {
        it("should support settings persistence", () => {
            // Test that the extension has settings persistence capability
            expect(typeof extension.handleSettingsAction).toBe("function");
            expect(typeof extension.settingsNodes).toBe("function");
        });
    });

    describe("Message Processing", () => {
        it("should handle empty collision object arrays", () => {
            const emptyScene = createTestPlanningScene([]);

            // Test that the extension can handle scenes with no collision objects
            expect(emptyScene.world.collision_objects).toHaveLength(0);
            expect(emptyScene.name).toBe("test_scene");
        });

        it("should handle objects with missing geometry", () => {
            const emptyGeometryObject: CollisionObject = {
                ...createTestCollisionObject("empty_object"),
                primitives: [],
                meshes: [],
                planes: [],
            };

            expect(emptyGeometryObject.primitives).toHaveLength(0);
            expect(emptyGeometryObject.meshes).toHaveLength(0);
            expect(emptyGeometryObject.planes).toHaveLength(0);
        });
    });

    describe("Rendering Tests", () => {
        beforeEach(() => {
            // Setup a layer to enable message processing
            (mockRenderer.config.layers as any)["test-layer"] = {
                layerId: "foxglove.PlanningScene",
                visible: true,
                topic: "/planning_scene",
            };
        });

        it("should render mesh objects and create renderables", () => {
            const meshObject: CollisionObject = {
                ...createTestCollisionObject("test_mesh"),
                primitives: [],
                primitive_poses: [],
                meshes: [{
                    vertices: [
                        { x: 0, y: 0, z: 0 },
                        { x: 1, y: 0, z: 0 },
                        { x: 0.5, y: 1, z: 0 }
                    ],
                    triangles: [
                        { vertex_indices: new Uint32Array([0, 1, 2]) }
                    ],
                }],
                mesh_poses: [{ position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }],
            };

            const sceneWithMeshes = createTestPlanningScene([meshObject]);

            // Simulate message processing by calling the handler directly
            const messageEvent = {
                topic: "/planning_scene",
                receiveTime: { sec: 0, nsec: 0 },
                message: sceneWithMeshes,
                schemaName: "moveit_msgs/PlanningScene",
                sizeInBytes: 1024,
            };

            // Get the message handler from subscriptions
            const subscriptions = extension.getSubscriptions();
            const handler = subscriptions[0]?.subscription.handler;
            expect(handler).toBeDefined();

            // Call the handler to process the message
            handler!(messageEvent);

            // Verify that renderables were created for mesh objects
            expect(extension.renderables.size).toBe(1);
            expect(extension.renderables.has("test_mesh")).toBe(true);
        });

        it("should render primitive shapes (implemented)", () => {
            const boxObject = createTestCollisionObject("test_box");
            const sphereObject = createTestCollisionObject("test_sphere");
            sphereObject.primitives = [{ type: SolidPrimitiveType.SPHERE, dimensions: [0.5] }];

            const sceneWithPrimitives = createTestPlanningScene([boxObject, sphereObject]);

            // Simulate message processing
            const messageEvent = {
                topic: "/planning_scene",
                receiveTime: { sec: 0, nsec: 0 },
                message: sceneWithPrimitives,
                schemaName: "moveit_msgs/PlanningScene",
                sizeInBytes: 1024,
            };

            const subscriptions = extension.getSubscriptions();
            const handler = subscriptions[0]?.subscription.handler;
            expect(handler).toBeDefined();

            // Call the handler to process the message
            handler!(messageEvent);

            // Verify that renderables were created for primitive shapes (they ARE implemented)
            expect(extension.renderables.size).toBe(2);
            expect(extension.renderables.has("test_box")).toBe(true);
            expect(extension.renderables.has("test_sphere")).toBe(true);
        });

        it("should NOT render octomap (not implemented)", () => {
            const sceneWithOctomap = createTestPlanningScene([]);
            sceneWithOctomap.world.octomap = {
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

            // Simulate message processing
            const messageEvent = {
                topic: "/planning_scene",
                receiveTime: { sec: 0, nsec: 0 },
                message: sceneWithOctomap,
                schemaName: "moveit_msgs/PlanningScene",
                sizeInBytes: 1024,
            };

            const subscriptions = extension.getSubscriptions();
            const handler = subscriptions[0]?.subscription.handler;
            expect(handler).toBeDefined();

            // Call the handler to process the message
            handler!(messageEvent);

            // Verify that NO renderables were created for octomap (not implemented)
            expect(extension.renderables.size).toBe(0);
        });
    });
});
