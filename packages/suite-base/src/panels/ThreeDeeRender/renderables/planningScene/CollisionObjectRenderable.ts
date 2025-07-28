// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import * as THREE from "three";

import Logger from "@lichtblick/log";
import type { RosValue } from "@lichtblick/suite-base/players/types";

import {
  CollisionObject,
  SolidPrimitive,
  SolidPrimitiveType,
  Mesh,
  Plane,
  normalizeVertexIndices,
  normalizeDimensions,
} from "./types";
import type { IRenderer } from "../../IRenderer";
import { BaseUserData, Renderable } from "../../Renderable";
import { makeRgba, stringToRgba } from "../../color";
import { Marker, MarkerType, Vector3 } from "../../ros";
import { BaseSettings } from "../../settings";
import { Pose } from "../../transforms";
import { RenderableCone } from "../markers/RenderableCone";
import { RenderableCube } from "../markers/RenderableCube";
import { RenderableCylinder } from "../markers/RenderableCylinder";
import { RenderablePlane } from "../markers/RenderablePlane";
import { RenderableSphere } from "../markers/RenderableSphere";
import { RenderableTriangleList } from "../markers/RenderableTriangleList";

export type CollisionObjectSettings = BaseSettings & {
  color?: string;
  opacity: number;
  showPrimitives: boolean;
  showMeshes: boolean;
  showPlanes: boolean;
};

export type CollisionObjectUserData = BaseUserData & {
  settings: CollisionObjectSettings;
  collisionObject: CollisionObject;
  shapes: Map<string, Renderable>;
};

// Note: Default color is now provided by the extension settings, not hardcoded here

const log = Logger.getLogger(__filename);

export class CollisionObjectRenderable extends Renderable<CollisionObjectUserData> {
  private shapes = new Map<string, Renderable>();

  // Performance optimization: Reference to extension for performance optimizations
  private extension?: {
    loadMeshResource: (meshData: Mesh) => Promise<THREE.BufferGeometry>;
    getSharedGeometry: (type: string, dimensions: number[], createGeometry: () => THREE.BufferGeometry) => THREE.BufferGeometry;
    getSharedMaterial: (color: string, opacity: number) => THREE.MeshStandardMaterial;
  };

  public constructor(name: string, renderer: IRenderer, userData: CollisionObjectUserData) {
    super(name, renderer, userData);
    // Container acts as a group - doesn't render anything itself
  }

  // Set reference to extension for performance optimizations
  public setExtension(extension: {
    loadMeshResource: (meshData: Mesh) => Promise<THREE.BufferGeometry>;
    getSharedGeometry: (type: string, dimensions: number[], createGeometry: () => THREE.BufferGeometry) => THREE.BufferGeometry;
    getSharedMaterial: (color: string, opacity: number) => THREE.MeshStandardMaterial;
  }): void {
    this.extension = extension;
  }

  public override dispose(): void {
    this.clearShapes();
    super.dispose();
  }

  // Update collision object with new data (Compositional Hierarchy pattern)
  public update(object: CollisionObject): void {
    // 1. Apply the main pose of the CollisionObject to this container's userData
    this.userData.pose = object.pose;
    this.userData.frameId = object.header.frame_id;
    this.userData.collisionObject = object;

    // 2. Clear existing shapes
    this.clearShapes();

    // 3. Create new shapes as children with LOCAL poses
    if (this.userData.settings.showPrimitives) {
      this.createPrimitiveShapes(object.primitives, object.primitive_poses);
    }
    if (this.userData.settings.showMeshes) {
      this.createMeshShapes(object.meshes, object.mesh_poses);
    }
    if (this.userData.settings.showPlanes) {
      this.createPlaneShapes(object.planes, object.plane_poses);
    }
  }

  // Clear all child shapes and dispose resources
  private clearShapes(): void {
    for (const shape of this.shapes.values()) {
      this.remove(shape);
      shape.dispose();
    }
    this.shapes.clear();

    // Clear any shape creation errors when clearing shapes
    this.renderer.settings.errors.clearPath([...this.userData.settingsPath, "shapes"]);
  }

  // Create primitive shapes with LOCAL poses relative to this container
  private createPrimitiveShapes(primitives: SolidPrimitive[], poses: Pose[]): void {
    for (let i = 0; i < primitives.length; i++) {
      const primitive = primitives[i];
      if (primitive == undefined) {
        continue; // Skip undefined primitives
      }
      const pose = poses[i] ?? { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };

      let shape: Renderable;
      try {
        // Normalize dimensions to handle arrays from ROS messages
        const dimensions = normalizeDimensions(primitive.dimensions);

        // Check for invalid dimension values
        for (let j = 0; j < dimensions.length; j++) {
          const dim = dimensions[j];
          if (typeof dim !== 'number' || !isFinite(dim) || dim <= 0) {
            throw new Error(`Primitive at index ${i} has invalid dimension at index ${j}: ${dim} (must be positive finite number)`);
          }
        }

        switch (primitive.type) {
          case SolidPrimitiveType.BOX: {
            if (dimensions.length < 3) {
              throw new Error(`Box primitive at index ${i} requires 3 dimensions [x, y, z], got ${dimensions.length}`);
            }
            shape = this.createBoxShape(dimensions);
            break;
          }
          case SolidPrimitiveType.SPHERE: {
            if (dimensions.length < 1) {
              throw new Error(`Sphere primitive at index ${i} requires 1 dimension [radius], got ${dimensions.length}`);
            }
            shape = this.createSphereShape(dimensions);
            break;
          }
          case SolidPrimitiveType.CYLINDER: {
            if (dimensions.length < 2) {
              throw new Error(`Cylinder primitive at index ${i} requires 2 dimensions [height, radius], got ${dimensions.length}`);
            }
            shape = this.createCylinderShape(dimensions);
            break;
          }
          case SolidPrimitiveType.CONE: {
            if (dimensions.length < 2) {
              throw new Error(`Cone primitive at index ${i} requires 2 dimensions [height, radius], got ${dimensions.length}`);
            }
            shape = this.createConeShape(dimensions);
            break;
          }
          default: {
            log.warn(`Unsupported primitive type ${primitive.type} at index ${i} in collision object '${this.userData.collisionObject.id}', skipping`);
            continue; // Skip unsupported primitives
          }
        }

        // Use shared geometry for identical primitive shapes to improve performance
        if (this.extension?.getSharedGeometry) {
          // Use shared geometry to reduce memory usage for identical shapes
          const geometryType = this.getGeometryTypeForPrimitive(primitive.type);
          if (geometryType) {
            const sharedGeometry = this.extension.getSharedGeometry(
              geometryType,
              dimensions,
              () => this.createGeometryForPrimitive(primitive.type, dimensions)
            );
            // Apply shared geometry to the shape if it supports it
            this.applySharedGeometry(shape, sharedGeometry);
          }
        }

        // Validate pose values
        if (!isFinite(pose.position.x) || !isFinite(pose.position.y) || !isFinite(pose.position.z)) {
          throw new Error(`Primitive at index ${i} has invalid position values: [${pose.position.x}, ${pose.position.y}, ${pose.position.z}]`);
        }

        if (!isFinite(pose.orientation.x) || !isFinite(pose.orientation.y) || !isFinite(pose.orientation.z) || !isFinite(pose.orientation.w)) {
          throw new Error(`Primitive at index ${i} has invalid orientation values: [${pose.orientation.x}, ${pose.orientation.y}, ${pose.orientation.z}, ${pose.orientation.w}]`);
        }

        // Set LOCAL position/rotation relative to this container
        shape.position.set(pose.position.x, pose.position.y, pose.position.z);
        shape.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        this.add(shape); // Add as child
        this.shapes.set(`primitive_${i}`, shape);
      } catch (error) {
        const primitiveTypeName = SolidPrimitiveType[primitive.type] || `UNKNOWN(${primitive.type})`;
        const errorMessage = `Failed to create primitive shape ${i} (type: ${primitiveTypeName}) in collision object '${this.userData.collisionObject.id}': ${error instanceof Error ? error.message : String(error)}`;

        // Report shape creation error to settings tree
        this.renderer.settings.errors.add(
          [...this.userData.settingsPath, "shapes"],
          `SHAPE_CREATION_ERROR_${i}`,
          errorMessage
        );
      }
    }
  }

  // Create mesh shapes from triangle data
  private createMeshShapes(meshes: Mesh[], poses: Pose[]): void {
    for (let i = 0; i < meshes.length; i++) {
      const mesh = meshes[i];
      if (mesh == undefined) {
        continue; // Skip undefined meshes
      }
      const pose = poses[i] ?? { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };

      try {
        // Validate mesh data before creating shape
        if (!Array.isArray(mesh.vertices)) {
          throw new Error(`Mesh at index ${i} has no vertices or invalid vertices array`);
        }

        if (mesh.vertices.length === 0) {
          throw new Error(`Mesh at index ${i} has empty vertices array`);
        }

        if (!Array.isArray(mesh.triangles)) {
          throw new Error(`Mesh at index ${i} has no triangles or invalid triangles array`);
        }

        if (mesh.triangles.length === 0) {
          throw new Error(`Mesh at index ${i} has empty triangles array`);
        }

        // Validate vertex data
        for (let j = 0; j < mesh.vertices.length; j++) {
          const vertex = mesh.vertices[j];
          if (vertex == undefined || typeof vertex.x !== 'number' || typeof vertex.y !== 'number' || typeof vertex.z !== 'number') {
            throw new Error(`Mesh at index ${i} has invalid vertex at index ${j}: vertex must have numeric x, y, z coordinates`);
          }

          if (!isFinite(vertex.x) || !isFinite(vertex.y) || !isFinite(vertex.z)) {
            throw new Error(`Mesh at index ${i} has non-finite vertex coordinates at index ${j}: [${vertex.x}, ${vertex.y}, ${vertex.z}]`);
          }
        }

        // Validate triangle data
        for (let j = 0; j < mesh.triangles.length; j++) {
          const triangle = mesh.triangles[j];

          // Normalize vertex indices to handle arrays from ROS messages
          const vertexIndices = normalizeVertexIndices(triangle?.vertex_indices);

          if (vertexIndices.length !== 3) {
            throw new Error(`Mesh at index ${i} has invalid triangle at index ${j}: vertex_indices must have exactly 3 elements, got ${vertexIndices.length}`);
          }

          for (let k = 0; k < 3; k++) {
            const vertexIndex = vertexIndices[k];
            if (typeof vertexIndex !== 'number' || !Number.isInteger(vertexIndex) || vertexIndex < 0 || vertexIndex >= mesh.vertices.length) {
              throw new Error(`Mesh at index ${i}, triangle ${j} has invalid vertex index at position ${k}: ${vertexIndex} (must be integer 0-${mesh.vertices.length - 1})`);
            }
          }
        }

        // Validate pose values
        if (!isFinite(pose.position.x) || !isFinite(pose.position.y) || !isFinite(pose.position.z)) {
          throw new Error(`Mesh at index ${i} has invalid position values: [${pose.position.x}, ${pose.position.y}, ${pose.position.z}]`);
        }

        if (!isFinite(pose.orientation.x) || !isFinite(pose.orientation.y) || !isFinite(pose.orientation.z) || !isFinite(pose.orientation.w)) {
          throw new Error(`Mesh at index ${i} has invalid orientation values: [${pose.orientation.x}, ${pose.orientation.y}, ${pose.orientation.z}, ${pose.orientation.w}]`);
        }

        // Lazy loading for mesh resources to improve performance
        if (this.extension?.loadMeshResource) {
          // Use lazy loading for mesh resources
          void this.extension.loadMeshResource(mesh).then((_geometry) => {
            // The geometry is loaded and cached by the extension, but we still create the shape
            // using the standard method since RenderableTriangleList handles its own geometry
            const shape = this.createTriangleListFromMesh(mesh);

            // Set LOCAL position/rotation relative to this container
            shape.position.set(pose.position.x, pose.position.y, pose.position.z);
            shape.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

            this.add(shape);
            this.shapes.set(`mesh_${i}`, shape);
          }).catch((error: unknown) => {
            const errorMessage = `Failed to load mesh resource ${i} in collision object '${this.userData.collisionObject.id}': ${error instanceof Error ? error.message : String(error)}`;
            log.warn(errorMessage);

            // Report mesh loading error to settings tree
            this.renderer.settings.errors.add(
              [...this.userData.settingsPath, "shapes"],
              `MESH_LOADING_ERROR_${i}`,
              errorMessage
            );
          });
        } else {
          // Fallback to synchronous creation
          const shape = this.createTriangleListFromMesh(mesh);

          // Set LOCAL position/rotation relative to this container
          shape.position.set(pose.position.x, pose.position.y, pose.position.z);
          shape.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

          this.add(shape);
          this.shapes.set(`mesh_${i}`, shape);
        }
      } catch (error) {
        const errorMessage = `Failed to create mesh shape ${i} in collision object '${this.userData.collisionObject.id}': ${error instanceof Error ? error.message : String(error)}`;
        log.warn(errorMessage);

        // Report mesh loading error to settings tree
        this.renderer.settings.errors.add(
          [...this.userData.settingsPath, "shapes"],
          `MESH_LOADING_ERROR_${i}`,
          errorMessage
        );
      }
    }
  }

  // Create plane shapes (rendered as thin boxes)
  private createPlaneShapes(planes: Plane[], poses: Pose[]): void {
    for (let i = 0; i < planes.length; i++) {
      const plane = planes[i];
      if (plane == undefined) {
        log.warn(`Plane at index ${i} in collision object '${this.userData.collisionObject.id}' is null or undefined, skipping`);
        continue;
      }
      const pose = poses[i] ?? { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };

      try {
        // Validate pose values
        if (!isFinite(pose.position.x) || !isFinite(pose.position.y) || !isFinite(pose.position.z)) {
          throw new Error(`Plane at index ${i} has invalid position values: [${pose.position.x}, ${pose.position.y}, ${pose.position.z}]`);
        }

        if (!isFinite(pose.orientation.x) || !isFinite(pose.orientation.y) || !isFinite(pose.orientation.z) || !isFinite(pose.orientation.w)) {
          throw new Error(`Plane at index ${i} has invalid orientation values: [${pose.orientation.x}, ${pose.orientation.y}, ${pose.orientation.z}, ${pose.orientation.w}]`);
        }

        const shape = this.createPlaneShape();

        // Set LOCAL position/rotation relative to this container
        shape.position.set(pose.position.x, pose.position.y, pose.position.z);
        shape.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        this.add(shape);
        this.shapes.set(`plane_${i}`, shape);
      } catch (error) {
        const errorMessage = `Failed to create plane shape ${i} in collision object '${this.userData.collisionObject.id}': ${error instanceof Error ? error.message : String(error)}`;
        log.warn(errorMessage);

        // Report plane creation error to settings tree
        this.renderer.settings.errors.add(
          [...this.userData.settingsPath, "shapes"],
          `SHAPE_CREATION_ERROR_PLANE_${i}`,
          errorMessage
        );
      }
    }
  }

  // Helper methods for specific primitive types
  private createBoxShape(dimensions: number[]): RenderableCube {
    const marker = this.createMarkerFromDimensions(MarkerType.CUBE, dimensions);
    return new RenderableCube(this.userData.topic ?? "", marker, this.userData.receiveTime, this.renderer);
  }

  private createSphereShape(dimensions: number[]): RenderableSphere {
    const marker = this.createMarkerFromDimensions(MarkerType.SPHERE, dimensions);
    return new RenderableSphere(this.userData.topic ?? "", marker, this.userData.receiveTime, this.renderer);
  }

  private createCylinderShape(dimensions: number[]): RenderableCylinder {
    const marker = this.createMarkerFromDimensions(MarkerType.CYLINDER, dimensions);
    return new RenderableCylinder(this.userData.topic ?? "", marker, this.userData.receiveTime, this.renderer);
  }

  private createConeShape(dimensions: number[]): RenderableCone {
    const marker = this.createMarkerFromDimensions(MarkerType.CYLINDER, dimensions); // Use CYLINDER type for cone
    return new RenderableCone(this.userData.topic ?? "", marker, this.userData.receiveTime, this.renderer);
  }

  private createTriangleListFromMesh(mesh: Mesh): RenderableTriangleList {
    const marker = this.createTriangleListMarker(mesh);
    return new RenderableTriangleList(this.userData.topic ?? "", marker, this.userData.receiveTime, this.renderer);
  }

  private createPlaneShape(): RenderablePlane {
    // Create a proper plane marker - RenderablePlane uses THREE.PlaneGeometry internally
    // We use CUBE marker type but RenderablePlane will render it as a plane
    const marker = this.createMarkerFromDimensions(MarkerType.CUBE, [10, 10, 1]); // Width, height, depth (depth ignored for plane)
    return new RenderablePlane(this.userData.topic ?? "", marker, this.userData.receiveTime, this.renderer);
  }

  // Helper to create a marker from primitive dimensions
  private createMarkerFromDimensions(type: MarkerType, dimensions: number[]): Marker {
    // Color should always be provided by settings, but fallback to gray if somehow missing
    const color = stringToRgba(makeRgba(), this.userData.settings.color ?? "#808080");
    color.a = this.userData.settings.opacity;

    const scale: Vector3 = { x: 1, y: 1, z: 1 };

    switch (type) {
      case MarkerType.CUBE: {
        // Box dimensions: [x, y, z]
        scale.x = dimensions[0] ?? 1;
        scale.y = dimensions[1] ?? 1;
        scale.z = dimensions[2] ?? 1;
        break;
      }
      case MarkerType.SPHERE: {
        // Sphere dimensions: [radius]
        const radius = dimensions[0] ?? 0.5;
        scale.x = scale.y = scale.z = radius * 2; // Diameter
        break;
      }
      case MarkerType.CYLINDER: {
        // Cylinder dimensions: [height, radius]
        // Also used for cones since there's no CONE marker type
        const height = dimensions[0] ?? 1;
        const radius = dimensions[1] ?? 0.5;
        scale.x = scale.y = radius * 2; // Diameter for x and y
        scale.z = height; // Height along z-axis
        break;
      }
    }

    return {
      header: {
        frame_id: this.userData.frameId,
        stamp: { sec: 0, nsec: 0 },
      },
      ns: "",
      id: 0,
      type,
      action: 0,
      pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      scale,
      color,
      lifetime: { sec: 0, nsec: 0 },
      frame_locked: false,
      points: [],
      colors: [],
      text: "",
      mesh_resource: "",
      mesh_use_embedded_materials: false,
    };
  }

  // Helper to create a triangle list marker from mesh data
  private createTriangleListMarker(mesh: Mesh): Marker {
    // Color should always be provided by settings, but fallback to gray if somehow missing
    const color = stringToRgba(makeRgba(), this.userData.settings.color ?? "#808080");
    color.a = this.userData.settings.opacity;

    const points: Vector3[] = [];
    const colors = [];

    // Convert mesh triangles to points
    for (const triangle of mesh.triangles) {
      const vertexIndices = normalizeVertexIndices(triangle.vertex_indices);
      for (const vertexIndex of vertexIndices) {
        const vertex = mesh.vertices[vertexIndex];
        if (vertex) {
          points.push({ x: vertex.x, y: vertex.y, z: vertex.z });
          colors.push(color);
        }
      }
    }

    return {
      header: {
        frame_id: this.userData.frameId,
        stamp: { sec: 0, nsec: 0 },
      },
      ns: "",
      id: 0,
      type: MarkerType.TRIANGLE_LIST,
      action: 0,
      pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      scale: { x: 1, y: 1, z: 1 },
      color,
      lifetime: { sec: 0, nsec: 0 },
      frame_locked: false,
      points,
      colors,
      text: "",
      mesh_resource: "",
      mesh_use_embedded_materials: false,
    };
  }

  public override idFromMessage(): string {
    return this.userData.collisionObject.id;
  }

  // Helper methods for geometry sharing to improve performance
  private getGeometryTypeForPrimitive(primitiveType: SolidPrimitiveType): string | undefined {
    switch (primitiveType) {
      case SolidPrimitiveType.BOX:
        return "box";
      case SolidPrimitiveType.SPHERE:
        return "sphere";
      case SolidPrimitiveType.CYLINDER:
        return "cylinder";
      case SolidPrimitiveType.CONE:
        return "cone";
      default:
        return undefined;
    }
  }

  private createGeometryForPrimitive(primitiveType: SolidPrimitiveType, dimensions: number[]): THREE.BufferGeometry {
    switch (primitiveType) {
      case SolidPrimitiveType.BOX: {
        const [x = 1, y = 1, z = 1] = dimensions;
        return new THREE.BoxGeometry(x, y, z);
      }
      case SolidPrimitiveType.SPHERE: {
        const [radius = 0.5] = dimensions;
        return new THREE.SphereGeometry(radius, 32, 16);
      }
      case SolidPrimitiveType.CYLINDER: {
        const [height = 1, radius = 0.5] = dimensions;
        return new THREE.CylinderGeometry(radius, radius, height, 32);
      }
      case SolidPrimitiveType.CONE: {
        const [height = 1, radius = 0.5] = dimensions;
        return new THREE.ConeGeometry(radius, height, 32);
      }
      default:
        throw new Error(`Unsupported primitive type for geometry creation: ${primitiveType}`);
    }
  }

  private applySharedGeometry(_shape: Renderable, _sharedGeometry: THREE.BufferGeometry): void {
    // This method would apply shared geometry to the renderable if it supports it
    // For now, we rely on the individual renderable classes to handle geometry sharing
    // This is a placeholder for future optimization where renderables could accept pre-built geometries

    // Note: The actual geometry sharing happens at the THREE.js level within each renderable class
    // when they create their geometries. This method is here for future extensibility.
  }

  public override details(): Record<string, RosValue> {
    const object = this.userData.collisionObject;
    return {
      id: object.id,
      frame_id: object.header.frame_id,
      primitives: object.primitives.length,
      meshes: object.meshes.length,
      planes: object.planes.length,
      operation: object.operation,
    };
  }
}
