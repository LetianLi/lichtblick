// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import * as THREE from "three";

import type { RosValue } from "@lichtblick/suite-base/players/types";

import {
  CollisionObject,
  SolidPrimitive,
  SolidPrimitiveType,
  Mesh,
  Plane,
  normalizeVertexIndices,
  normalizeDimensions,
  SolidPrimitiveDimension,
  getSolidPrimitiveDimensionNames,
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
};

export type CollisionObjectUserData = BaseUserData & {
  settings: CollisionObjectSettings;
  collisionObject: CollisionObject;
  shapes: Map<string, Renderable>;
};

export class CollisionObjectRenderable extends Renderable<CollisionObjectUserData> {
  public constructor(name: string, renderer: IRenderer, userData: CollisionObjectUserData) {
    super(name, renderer, userData);
    // Container acts as a group - doesn't render anything itself
  }

  public override dispose(): void {
    this.clearShapes();
    super.dispose();
  }

  // Update collision object with new data
  public update(object: CollisionObject): void {
    // 1. Apply the main pose of the CollisionObject to this container's userData
    this.userData.pose = object.pose;
    this.userData.frameId = object.header.frame_id;
    this.userData.collisionObject = object;

    // 2. Clear existing shapes
    this.clearShapes();

    // 3. Create new shapes as children with LOCAL poses
    this.createPrimitiveShapes(object.primitives, object.primitive_poses);
    this.createMeshShapes(object.meshes, object.mesh_poses);
    this.createPlaneShapes(object.planes, object.plane_poses);

    // After attempting to create all shapes, update the parent-level summary
    this.#updateShapeFailureSummary();
  }

  // Clear all child shapes and dispose resources
  private clearShapes(): void {
    for (const [key, shape] of this.userData.shapes.entries()) {
      this.remove(shape);
      shape.dispose();
      // Clear per-shape errors
      this.renderer.settings.errors.clearPath([...this.userData.settingsPath, key]);
    }
    this.userData.shapes.clear();

    // Note: per-shape errors are cleared above; no group node is used
  }

  // Create primitive shapes with LOCAL poses relative to this container
  private createPrimitiveShapes(primitives: SolidPrimitive[], poses: Pose[]): void {
    for (let i = 0; i < primitives.length; i++) {
      const primitive = primitives[i];
      try {
        if (primitive == undefined) {
          throw new Error(`Primitive is undefined`);
        }
        const pose = poses[i] ?? {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        };

        let shape: Renderable;
        // Normalize dimensions to handle arrays from ROS messages
        const dimensions = normalizeDimensions(primitive.dimensions);

        // Check for invalid dimension values
        for (let j = 0; j < dimensions.length; j++) {
          const dim = dimensions[j];
          if (typeof dim !== "number" || !isFinite(dim) || dim <= 0) {
            throw new Error(`Invalid dimension[${j}]: ${dim} (must be positive finite number)`);
          }
        }

        switch (primitive.type) {
          case SolidPrimitiveType.BOX: {
            if (dimensions.length < 3) {
              const expectedDims = getSolidPrimitiveDimensionNames(primitive.type);
              throw new Error(
                `Insufficient dimensions: Expected ${expectedDims.length} dimension${expectedDims.length > 1 ? "s" : ""}[${expectedDims.join(", ")}], got ${dimensions.length}`,
              );
            }
            shape = this.createBoxShape(dimensions);
            break;
          }
          case SolidPrimitiveType.SPHERE: {
            if (dimensions.length < 1) {
              const expectedDims = getSolidPrimitiveDimensionNames(primitive.type);
              throw new Error(
                `Insufficient dimensions: Expected ${expectedDims.length} dimension${expectedDims.length > 1 ? "s" : ""}[${expectedDims.join(", ")}], got ${dimensions.length}`,
              );
            }
            shape = this.createSphereShape(dimensions);
            break;
          }
          case SolidPrimitiveType.CYLINDER: {
            if (dimensions.length < 2) {
              const expectedDims = getSolidPrimitiveDimensionNames(primitive.type);
              throw new Error(
                `Insufficient dimensions: Expected ${expectedDims.length} dimension${expectedDims.length > 1 ? "s" : ""}[${expectedDims.join(", ")}], got ${dimensions.length}`,
              );
            }
            shape = this.createCylinderShape(dimensions);
            break;
          }
          case SolidPrimitiveType.CONE: {
            if (dimensions.length < 2) {
              const expectedDims = getSolidPrimitiveDimensionNames(primitive.type);
              throw new Error(
                `Insufficient dimensions: Expected ${expectedDims.length} dimension${expectedDims.length > 1 ? "s" : ""}[${expectedDims.join(", ")}], got ${dimensions.length}`,
              );
            }
            shape = this.createConeShape(dimensions);
            break;
          }
          default: {
            throw new Error(`Unsupported primitive type`);
          }
        }

        // Validate pose values
        if (
          !isFinite(pose.position.x) ||
          !isFinite(pose.position.y) ||
          !isFinite(pose.position.z)
        ) {
          throw new Error(
            `Invalid position: [${pose.position.x}, ${pose.position.y}, ${pose.position.z}]`,
          );
        }

        if (
          !isFinite(pose.orientation.x) ||
          !isFinite(pose.orientation.y) ||
          !isFinite(pose.orientation.z) ||
          !isFinite(pose.orientation.w)
        ) {
          throw new Error(
            `Invalid orientation: [${pose.orientation.x}, ${pose.orientation.y}, ${pose.orientation.z}, ${pose.orientation.w}]`,
          );
        }

        // Set LOCAL position/rotation relative to this container
        shape.position.set(pose.position.x, pose.position.y, pose.position.z);
        shape.quaternion.set(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
        );

        // Assign per-shape settings path and persisted visibility
        const shapeKey = `primitive_${i}`;
        shape.userData.settingsPath = [...this.userData.settingsPath, shapeKey];
        const instanceId = this.userData.settingsPath?.[1];
        const objectId = this.userData.collisionObject.id;
        const layerConfig = instanceId
          ? (this.renderer.config.layers[instanceId] as any)
          : undefined;
        const savedVisible = layerConfig?.collisionObjects?.[objectId]?.shapes?.[shapeKey]?.visible;
        const visible = savedVisible ?? (shape.userData.settings?.visible ?? true);
        if (shape.userData.settings) {
          shape.userData.settings.visible = Boolean(visible);
        }
        shape.visible = Boolean(visible);

        this.add(shape); // Add as child
        this.userData.shapes.set(shapeKey, shape);
      } catch (error) {
        const primitiveTypeName = primitive
          ? SolidPrimitiveType[primitive.type] || `UNKNOWN`
          : "UNDEFINED";
        const errorMessage = `Failed to create primitive (${primitiveTypeName}): ${
          error instanceof Error ? error.message : String(error)
        }`;

        // Report shape creation error to settings tree at per-shape path
        this.renderer.settings.errors.add(
          [...this.userData.settingsPath, `primitive_${i}`],
          `SHAPE_CREATION_ERROR_${i}`,
          errorMessage,
        );
        // Bubble a concise error to the collision object row as well
        this.renderer.settings.errors.add(
          this.userData.settingsPath,
          "SHAPE_ERRORS",
          this.#shapeFailureMessage(),
        );
        this.#updateShapeFailureSummary();
      }
    }
  }

  // Create mesh shapes from triangle data
  private createMeshShapes(meshes: Mesh[], poses: Pose[]): void {
    for (let i = 0; i < meshes.length; i++) {
      const mesh = meshes[i];
      try {
        if (mesh == undefined) {
          throw new Error(`Mesh is undefined`);
        }
        const pose = poses[i] ?? {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        };

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
          if (
            vertex == undefined ||
            typeof vertex.x !== "number" ||
            typeof vertex.y !== "number" ||
            typeof vertex.z !== "number"
          ) {
            throw new Error(
              `Mesh at index ${i} has invalid vertex at index ${j}: vertex must have numeric x, y, z coordinates`,
            );
          }

          if (!isFinite(vertex.x) || !isFinite(vertex.y) || !isFinite(vertex.z)) {
            throw new Error(
              `Mesh at index ${i} has non-finite vertex coordinates at index ${j}: [${vertex.x}, ${vertex.y}, ${vertex.z}]`,
            );
          }
        }

        // Validate triangle data
        for (let j = 0; j < mesh.triangles.length; j++) {
          const triangle = mesh.triangles[j];

          // Normalize vertex indices to handle arrays from ROS messages
          const vertexIndices = normalizeVertexIndices(triangle?.vertex_indices);

          if (vertexIndices.length !== 3) {
            throw new Error(
              `Mesh at index ${i} has invalid triangle at index ${j}: vertex_indices must have exactly 3 elements, got ${vertexIndices.length}`,
            );
          }

          for (let k = 0; k < 3; k++) {
            const vertexIndex = vertexIndices[k];
            if (
              typeof vertexIndex !== "number" ||
              !Number.isInteger(vertexIndex) ||
              vertexIndex < 0 ||
              vertexIndex >= mesh.vertices.length
            ) {
              throw new Error(
                `Mesh at index ${i}, triangle ${j} has invalid vertex index at position ${k}: ${vertexIndex} (must be integer 0-${mesh.vertices.length - 1})`,
              );
            }
          }
        }

        // Validate pose values
        if (
          !isFinite(pose.position.x) ||
          !isFinite(pose.position.y) ||
          !isFinite(pose.position.z)
        ) {
          throw new Error(
            `Mesh at index ${i} has invalid position values: [${pose.position.x}, ${pose.position.y}, ${pose.position.z}]`,
          );
        }

        if (
          !isFinite(pose.orientation.x) ||
          !isFinite(pose.orientation.y) ||
          !isFinite(pose.orientation.z) ||
          !isFinite(pose.orientation.w)
        ) {
          throw new Error(
            `Mesh at index ${i} has invalid orientation values: [${pose.orientation.x}, ${pose.orientation.y}, ${pose.orientation.z}, ${pose.orientation.w}]`,
          );
        }

        // Use triangle list marker rendering for meshes
        const shape = this.createTriangleListFromMesh(mesh);

        // Assign per-shape settings path and persisted visibility
        const shapeKey = `mesh_${i}`;
        shape.userData.settingsPath = [...this.userData.settingsPath, shapeKey];
        const instanceId = this.userData.settingsPath?.[1];
        const objectId = this.userData.collisionObject.id;
        const layerConfig = instanceId
          ? (this.renderer.config.layers[instanceId] as any)
          : undefined;
        const savedVisible =
          layerConfig?.collisionObjects?.[objectId]?.shapes?.[shapeKey]?.visible;
        const visible = savedVisible ?? (shape.userData.settings?.visible ?? true);
        if (shape.userData.settings) {
          shape.userData.settings.visible = Boolean(visible);
        }
        shape.visible = Boolean(visible);

        // Set LOCAL position/rotation relative to this container
        shape.position.set(pose.position.x, pose.position.y, pose.position.z);
        shape.quaternion.set(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
        );

        this.add(shape);
        this.userData.shapes.set(shapeKey, shape);
      } catch (error) {
          const errorMessage = `Failed to create mesh: ${
            error instanceof Error ? error.message : String(error)
          }`;

        // Report mesh loading error to settings tree at per-shape path
        this.renderer.settings.errors.add(
          [...this.userData.settingsPath, `mesh_${i}`],
          `MESH_LOADING_ERROR_${i}`,
          errorMessage,
        );
        // Bubble concise error to collision object row
        this.renderer.settings.errors.add(
          this.userData.settingsPath,
          "SHAPE_ERRORS",
          this.#shapeFailureMessage(),
        );
        this.#updateShapeFailureSummary();
      }
    }
  }

  // Create plane shapes (rendered as thin boxes)
  private createPlaneShapes(planes: Plane[], poses: Pose[]): void {
    for (let i = 0; i < planes.length; i++) {
      const plane = planes[i];
      try {
        if (plane == undefined) {
          throw new Error(`Plane is undefined`);
        }
        const pose = poses[i] ?? {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        };

        // Extract and validate plane coefficients
        const a = plane.coef[0];
        const b = plane.coef[1];
        const c = plane.coef[2];
        const d = plane.coef[3];

        // Check for invalid coefficient values (NaN, Infinity, etc.)
        if (!isFinite(a) || !isFinite(b) || !isFinite(c) || !isFinite(d)) {
          throw new Error(`Invalid coefficient: [${a}, ${b}, ${c}, ${d}] (must be finite numbers)`);
        }

        // Check for zero normal vector (all coefficients zero)
        if (a === 0 && b === 0 && c === 0) {
          throw new Error(
            `Zero normal vector: [${a}, ${b}, ${c}, ${d}] (at least one of a, b, c must be non-zero)`,
          );
        }

        // Validate pose values
        if (
          !isFinite(pose.position.x) ||
          !isFinite(pose.position.y) ||
          !isFinite(pose.position.z)
        ) {
          throw new Error(
            `Invalid position: [${pose.position.x}, ${pose.position.y}, ${pose.position.z}]`,
          );
        }

        if (
          !isFinite(pose.orientation.x) ||
          !isFinite(pose.orientation.y) ||
          !isFinite(pose.orientation.z) ||
          !isFinite(pose.orientation.w)
        ) {
          throw new Error(
            `Invalid orientation: [${pose.orientation.x}, ${pose.orientation.y}, ${pose.orientation.z}, ${pose.orientation.w}]`,
          );
        }

        const shape = this.createPlaneShape();

        // Normal vector and magnitude
        const nLen = Math.hypot(a, b, c);
        const normal = new THREE.Vector3(a, b, c).divideScalar(nLen);

        // Rotate +Z to plane normal
        const planeQuaternion = new THREE.Quaternion().setFromUnitVectors(
          new THREE.Vector3(0, 0, 1),
          normal,
        );

        // Combine pose orientation with plane orientation: pose * plane
        const poseQuat = new THREE.Quaternion(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
        );
        const combinedQuaternion = poseQuat.clone().multiply(planeQuaternion);
        shape.quaternion.copy(combinedQuaternion);

        // Apply plane offset along normal: offset = -(d / |n|) * n, rotated by pose orientation
        const offsetLocal = normal.clone().multiplyScalar(-d / nLen).applyQuaternion(poseQuat);
        shape.position.set(
          pose.position.x + offsetLocal.x,
          pose.position.y + offsetLocal.y,
          pose.position.z + offsetLocal.z,
        );

        // Assign per-shape settings path and persisted visibility
        const shapeKey = `plane_${i}`;
        shape.userData.settingsPath = [...this.userData.settingsPath, shapeKey];
        const instanceId = this.userData.settingsPath?.[1];
        const objectId = this.userData.collisionObject.id;
        const layerConfig = instanceId
          ? (this.renderer.config.layers[instanceId] as any)
          : undefined;
        const savedVisible = layerConfig?.collisionObjects?.[objectId]?.shapes?.[shapeKey]?.visible;
        const visible = savedVisible ?? (shape.userData.settings?.visible ?? true);
        if (shape.userData.settings) {
          shape.userData.settings.visible = Boolean(visible);
        }
        shape.visible = Boolean(visible);

        this.add(shape);
        this.userData.shapes.set(shapeKey, shape);
      } catch (error) {
        const errorMessage = `Failed to create plane: ${
          error instanceof Error ? error.message : String(error)
        }`;

        // Report plane creation error to settings tree at per-shape path
        this.renderer.settings.errors.add(
          [...this.userData.settingsPath, `plane_${i}`],
          `SHAPE_CREATION_ERROR_PLANE_${i}`,
          errorMessage,
        );
        // Bubble concise error to collision object row
        this.renderer.settings.errors.add(
          this.userData.settingsPath,
          "SHAPE_ERRORS",
          this.#shapeFailureMessage(),
        );
        this.#updateShapeFailureSummary();
      }
    }
  }

  // Build the summary message for parent row
  #shapeFailureMessage(): string {
    const obj = this.userData.collisionObject;
    const total = obj.primitives.length + obj.meshes.length + obj.planes.length;
    const failed = this.#countFailedShapes();
    return `${failed} of ${total} shapes failed to render`;
  }

  #countFailedShapes(): number {
    const base = this.userData.settingsPath;
    const obj = this.userData.collisionObject;
    let failed = 0;
    for (let i = 0; i < obj.primitives.length; i++) {
      if (this.renderer.settings.errors.errors.errorAtPath([...base, `primitive_${i}`]) != undefined) {
        failed++;
      }
    }
    for (let i = 0; i < obj.meshes.length; i++) {
      if (this.renderer.settings.errors.errors.errorAtPath([...base, `mesh_${i}`]) != undefined) {
        failed++;
      }
    }
    for (let i = 0; i < obj.planes.length; i++) {
      if (this.renderer.settings.errors.errors.errorAtPath([...base, `plane_${i}`]) != undefined) {
        failed++;
      }
    }
    return failed;
  }

  #updateShapeFailureSummary(): void {
    const failed = this.#countFailedShapes();
    const obj = this.userData.collisionObject;
    const total = obj.primitives.length + obj.meshes.length + obj.planes.length;
    if (failed > 0) {
      this.renderer.settings.errors.add(
        this.userData.settingsPath,
        "SHAPE_ERRORS",
        `${failed} of ${total} shapes failed to render`,
      );
    } else {
      // If no failures remain, clear summary (does not touch per-shape errors)
      this.renderer.settings.errors.remove(this.userData.settingsPath, "SHAPE_ERRORS");
    }
  }

  // Helper methods for specific primitive types
  private createBoxShape(dimensions: number[]): RenderableCube {
    const marker = this.createMarkerFromDimensions(MarkerType.CUBE, dimensions);
    return new RenderableCube(
      this.userData.topic ?? "",
      marker,
      this.userData.receiveTime,
      this.renderer,
    );
  }

  private createSphereShape(dimensions: number[]): RenderableSphere {
    const marker = this.createMarkerFromDimensions(MarkerType.SPHERE, dimensions);
    return new RenderableSphere(
      this.userData.topic ?? "",
      marker,
      this.userData.receiveTime,
      this.renderer,
    );
  }

  private createCylinderShape(dimensions: number[]): RenderableCylinder {
    const marker = this.createMarkerFromDimensions(MarkerType.CYLINDER, dimensions);
    return new RenderableCylinder(
      this.userData.topic ?? "",
      marker,
      this.userData.receiveTime,
      this.renderer,
    );
  }

  private createConeShape(dimensions: number[]): RenderableCone {
    const marker = this.createMarkerFromDimensions(MarkerType.CYLINDER, dimensions); // Use CYLINDER type for cone
    return new RenderableCone(
      this.userData.topic ?? "",
      marker,
      this.userData.receiveTime,
      this.renderer,
    );
  }

  private createTriangleListFromMesh(mesh: Mesh): RenderableTriangleList {
    const marker = this.createTriangleListMarker(mesh);
    return new RenderableTriangleList(
      this.userData.topic ?? "",
      marker,
      this.userData.receiveTime,
      this.renderer,
    );
  }

  private createPlaneShape(): RenderablePlane {
    // Create a proper plane marker - RenderablePlane uses THREE.PlaneGeometry internally
    // We use CUBE marker type but RenderablePlane will render it as a plane
    // The plane equation ax + by + cz + d = 0 will be handled by the pose orientation
    const marker = this.createMarkerFromDimensions(MarkerType.CUBE, [10, 10, 1]); // Width, height, depth (depth ignored for plane)
    return new RenderablePlane(
      this.userData.topic ?? "",
      marker,
      this.userData.receiveTime,
      this.renderer,
    );
  }

  // Helper to create a marker from primitive dimensions
  private createMarkerFromDimensions(type: MarkerType, dimensions: number[]): Marker {
    // Color should always be provided by settings, but fallback to gray if somehow missing
    const color = stringToRgba(makeRgba(), this.userData.settings.color ?? "#808080");
    color.a = this.userData.settings.opacity;

    const scale: Vector3 = { x: 1, y: 1, z: 1 };

    switch (type) {
      case MarkerType.CUBE: {
        scale.x = dimensions[SolidPrimitiveDimension.BOX_X] ?? 1;
        scale.y = dimensions[SolidPrimitiveDimension.BOX_Y] ?? 1;
        scale.z = dimensions[SolidPrimitiveDimension.BOX_Z] ?? 1;
        break;
      }
      case MarkerType.SPHERE: {
        const radius = dimensions[SolidPrimitiveDimension.SPHERE_RADIUS] ?? 0.5;
        scale.x = scale.y = scale.z = radius * 2; // Diameter
        break;
      }
      case MarkerType.CYLINDER: {
        // Also used for cones since there's no CONE marker type
        const height = dimensions[SolidPrimitiveDimension.CYLINDER_HEIGHT] ?? 1;
        const radius = dimensions[SolidPrimitiveDimension.CYLINDER_RADIUS] ?? 0.5;
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
