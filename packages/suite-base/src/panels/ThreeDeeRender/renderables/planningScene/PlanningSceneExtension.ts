// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { t } from "i18next";
import * as _ from "lodash-es";

import { filterMap } from "@lichtblick/den/collection";
import Logger from "@lichtblick/log";
import { toNanoSec } from "@lichtblick/rostime";
import { SettingsTreeAction, SettingsTreeFields, SettingsTreeChildren } from "@lichtblick/suite";

import {
  CollisionObjectRenderable,
  CollisionObjectUserData,
  CollisionObjectLayerConfig,
} from "./CollisionObjectRenderable";
import {
  PlanningScene,
  GetPlanningSceneRequest,
  GetPlanningSceneResponse,
  DEFAULT_PLANNING_SCENE_SERVICE,
  DEFAULT_PLANNING_SCENE_TOPIC,
  PLANNING_SCENE_DATATYPES,
  CollisionObject,
  CollisionObjectOperation,
  Mesh,
  SolidPrimitive,
  SolidPrimitiveType,
  RobotState,
  AttachedCollisionObject,
  createMinimalPlanningSceneComponents,
  normalizeDimensions,
  normalizeVertexIndices,
} from "./types";
import type { IRenderer, AnyRendererSubscription } from "../../IRenderer";
import { Renderable } from "../../Renderable";
import { SceneExtension, PartialMessageEvent, PartialMessage } from "../../SceneExtension";
import { SettingsTreeEntry, SettingsTreeNodeWithActionHandler } from "../../SettingsManager";
import { Header } from "../../ros";
import { BaseSettings, CustomLayerSettings } from "../../settings";
import { AnyFrameId, Pose } from "../../transforms";

// Error constants for settings tree
const SERVICE_ERROR = "SERVICE_ERROR";
const MESSAGE_PROCESSING_ERROR = "MESSAGE_PROCESSING_ERROR";

// Layer ID for custom layers
const LAYER_ID = "foxglove.PlanningScene";

// Default settings for topic-based planning scenes
export type PlanningSceneSettings = BaseSettings & {
  defaultColor: string;
  sceneOpacity: number;
  showCollisionObjects: boolean;
  showAttachedObjects: boolean;
  showOctomap: boolean;
};

// Settings for custom planning scene layers
export type LayerSettingsPlanningScene = CustomLayerSettings & {
  layerId: "foxglove.PlanningScene";
  topic: string;
  defaultColor: string;
  sceneOpacity: number;
  showCollisionObjects: boolean;
  showAttachedObjects: boolean;
  showOctomap: boolean;
  collisionObjects?: Record<string, CollisionObjectLayerConfig>;
};

const DEFAULT_PLANNING_SCENE_SETTINGS: PlanningSceneSettings = {
  visible: true,
  defaultColor: "#ffffff",
  sceneOpacity: 1.0,
  showCollisionObjects: true,
  showAttachedObjects: true,
  showOctomap: false,
};

const DEFAULT_CUSTOM_SETTINGS: LayerSettingsPlanningScene = {
  visible: true,
  frameLocked: true,
  label: "Planning Scene", // This will be overridden with i18n in the constructor
  instanceId: "invalid",
  layerId: LAYER_ID,
  topic: DEFAULT_PLANNING_SCENE_TOPIC,
  defaultColor: "#ffffff",
  sceneOpacity: 1.0,
  showCollisionObjects: true,
  showAttachedObjects: true,
  showOctomap: false,
};

const log = Logger.getLogger(__filename);

export class PlanningSceneExtension extends SceneExtension<CollisionObjectRenderable> {
  public static extensionId = "foxglove.PlanningScene";

  private currentScene?: PartialMessage<PlanningScene>;
  private serviceClient?: (service: string, request: unknown) => Promise<unknown>;
  private initialSceneFetched = false;
  private fetchingInitialScene = false;

  // Track current message context for creating renderables
  private topic = DEFAULT_PLANNING_SCENE_TOPIC;
  private receiveTime = 0n;
  private messageTime = 0n;
  private currentInstanceId?: string; // Track which layer instance is currently being processed. This should not be used because this class represents the extension and should have all layers.

  // Settings
  private settings: PlanningSceneSettings = { ...DEFAULT_PLANNING_SCENE_SETTINGS };

  // Performance optimization: Track object hashes for differential updates
  private objectHashes = new Map<string, string>();

  // Helper method to find the instance ID for a given topic
  private findInstanceIdForTopic(topic: string): string | undefined {
    // First try to find a layer with matching topic
    for (const [instanceId, layerConfig] of Object.entries(this.renderer.config.layers)) {
      if (layerConfig?.layerId === LAYER_ID) {
        const config = layerConfig as Partial<LayerSettingsPlanningScene>;
        if (config.topic === topic) {
          return instanceId;
        }
      }
    }

    // If no exact match, use the first available planning scene layer
    // (planning scene layers can listen to any planning scene topic)
    for (const [instanceId, layerConfig] of Object.entries(this.renderer.config.layers)) {
      if (layerConfig?.layerId === LAYER_ID) {
        return instanceId;
      }
    }

    return undefined;
  }

  // Helper method to load settings from layer configuration
  private loadSettingsFromConfig(instanceId?: string): PlanningSceneSettings {
    if (!instanceId || !this.renderer.config.layers[instanceId]) {
      return DEFAULT_PLANNING_SCENE_SETTINGS;
    }

    const layerConfig = this.renderer.config.layers[
      instanceId
    ] as Partial<LayerSettingsPlanningScene>;

    return {
      visible: layerConfig.visible ?? DEFAULT_PLANNING_SCENE_SETTINGS.visible,
      defaultColor: layerConfig.defaultColor ?? DEFAULT_PLANNING_SCENE_SETTINGS.defaultColor,
      sceneOpacity: layerConfig.sceneOpacity ?? DEFAULT_PLANNING_SCENE_SETTINGS.sceneOpacity,
      showCollisionObjects:
        layerConfig.showCollisionObjects ?? DEFAULT_PLANNING_SCENE_SETTINGS.showCollisionObjects,
      showAttachedObjects:
        layerConfig.showAttachedObjects ?? DEFAULT_PLANNING_SCENE_SETTINGS.showAttachedObjects,
      showOctomap: layerConfig.showOctomap ?? DEFAULT_PLANNING_SCENE_SETTINGS.showOctomap,
    };
  }

  // Helper method to extract color for a specific collision object from planning scene
  private getObjectColorFromScene(
    objectId: string,
  ): { color: string; opacity: number } | undefined {
    if (!this.currentScene?.object_colors) {
      log.warn(`No object colors found for object ${objectId}`);
      return undefined;
    }

    // Find the color for this specific object ID
    const objectColor = this.currentScene.object_colors.find((oc) => oc?.id === objectId);
    if (!objectColor?.color) {
      return undefined;
    }

    const { r, g, b, a } = objectColor.color;

    // Ensure all color components are defined and valid
    if (r == undefined || g == undefined || b == undefined || a == undefined) {
      log.warn(`Invalid color values for object ${objectId}: ${r}, ${g}, ${b}, ${a}`);
      return undefined;
    }

    // Convert RGBA values (0-1) to hex color string
    const rHex = Math.round(r * 255)
      .toString(16)
      .padStart(2, "0");
    const gHex = Math.round(g * 255)
      .toString(16)
      .padStart(2, "0");
    const bHex = Math.round(b * 255)
      .toString(16)
      .padStart(2, "0");
    const color = `#${rHex}${gHex}${bHex}`;

    return {
      color,
      opacity: a, // Alpha value is already in 0-1 range
    };
  }

  public constructor(
    renderer: IRenderer,
    serviceClient?: (service: string, request: unknown) => Promise<unknown>,
    name: string = PlanningSceneExtension.extensionId,
  ) {
    super(name, renderer);
    this.serviceClient = serviceClient;

    // Register custom layer action (if available)
    renderer.addCustomLayerAction({
      layerId: LAYER_ID,
      label: t("threeDee:addPlanningScene"),
      icon: "Cube",
      handler: this.#handleAddPlanningScene,
    });

    // Load existing planning scene layers from the config (if available)
    const planningSceneLayers = Object.entries(renderer.config.layers).filter(
      ([, entry]) => entry?.layerId === LAYER_ID,
    );

    for (const [instanceId, entry] of planningSceneLayers) {
      this.#updatePlanningSceneLayer(instanceId, entry as Partial<LayerSettingsPlanningScene>);
    }
  }

  // Compute hash for collision object to detect changes for performance optimization
  private computeObjectHash(object: CollisionObject): string {
    // Create a hash based on object properties that affect rendering
    // Use a more efficient hash computation to avoid JSON.stringify overhead
    const parts: string[] = [object.id, String(object.operation), object.header.frame_id];

    // Round pose values to avoid floating point precision issues
    const roundedPose = {
      x: Math.round(object.pose.position.x * 1000) / 1000,
      y: Math.round(object.pose.position.y * 1000) / 1000,
      z: Math.round(object.pose.position.z * 1000) / 1000,
      qx: Math.round(object.pose.orientation.x * 1000) / 1000,
      qy: Math.round(object.pose.orientation.y * 1000) / 1000,
      qz: Math.round(object.pose.orientation.z * 1000) / 1000,
      qw: Math.round(object.pose.orientation.w * 1000) / 1000,
    };
    parts.push(
      `${roundedPose.x},${roundedPose.y},${roundedPose.z},${roundedPose.qx},${roundedPose.qy},${roundedPose.qz},${roundedPose.qw}`,
    );

    // Hash primitives with rounded dimensions
    for (let i = 0; i < object.primitives.length; i++) {
      const p = object.primitives[i];
      const pose = object.primitive_poses[i];
      if (p && pose) {
        const roundedDims = p.dimensions.map((d) => Math.round(d * 1000) / 1000);
        const roundedPrimPose = {
          x: Math.round(pose.position.x * 1000) / 1000,
          y: Math.round(pose.position.y * 1000) / 1000,
          z: Math.round(pose.position.z * 1000) / 1000,
        };
        parts.push(
          `p${i}:${p.type}:${roundedDims.join(",")}:${roundedPrimPose.x},${roundedPrimPose.y},${roundedPrimPose.z}`,
        );
      }
    }

    // Hash meshes (use vertex/triangle counts and first few vertices for performance)
    for (let i = 0; i < object.meshes.length; i++) {
      const m = object.meshes[i];
      const pose = object.mesh_poses[i];
      if (m && pose) {
        // Create a simple structural hash of the mesh
        let structureHash = m.vertices.length * 1000 + m.triangles.length;
        // Sample first few vertices for uniqueness
        for (let j = 0; j < Math.min(m.vertices.length, 3); j++) {
          const v = m.vertices[j];
          if (v) {
            structureHash += Math.round(v.x * 100) + Math.round(v.y * 10) + Math.round(v.z);
          }
        }
        const roundedMeshPose = {
          x: Math.round(pose.position.x * 1000) / 1000,
          y: Math.round(pose.position.y * 1000) / 1000,
          z: Math.round(pose.position.z * 1000) / 1000,
        };
        parts.push(
          `m${i}:${structureHash}:${roundedMeshPose.x},${roundedMeshPose.y},${roundedMeshPose.z}`,
        );
      }
    }

    // Hash planes
    for (let i = 0; i < object.planes.length; i++) {
      const pose = object.plane_poses[i];
      if (pose) {
        const roundedPlanePose = {
          x: Math.round(pose.position.x * 1000) / 1000,
          y: Math.round(pose.position.y * 1000) / 1000,
          z: Math.round(pose.position.z * 1000) / 1000,
        };
        parts.push(`pl${i}:${roundedPlanePose.x},${roundedPlanePose.y},${roundedPlanePose.z}`);
      }
    }

    return parts.join("|");
  }

  // Check if object has changed since last update for performance optimization
  private hasObjectChanged(object: CollisionObject): boolean {
    const currentHash = this.computeObjectHash(object);
    const previousHash = this.objectHashes.get(object.id);

    if (previousHash !== currentHash) {
      this.objectHashes.set(object.id, currentHash);
      return true;
    }

    return false;
  }

  public override dispose(): void {
    // Clear performance optimization caches
    this.objectHashes.clear();

    // Clear any pending service calls by resetting the service client
    this.serviceClient = undefined;

    // Clear current scene data
    this.currentScene = undefined;

    // Reset state flags
    this.initialSceneFetched = false;
    this.fetchingInitialScene = false;

    // Clear all settings errors for this extension
    // Note: This extension creates custom layers, so we need to clear errors for each layer instance
    const instanceId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
    if (instanceId) {
      this.renderer.settings.errors.clearPath(["layers", instanceId, "collisionObjects"]);
    }

    // Clean up subscription and renderables (this calls dispose() on all renderables)
    super.dispose();
  }

  public override handleSettingsAction = (action: SettingsTreeAction): void => {
    this.#handleLayerSettingsAction(action);
  };

  public override settingsNodes(): SettingsTreeEntry[] {
    const entries: SettingsTreeEntry[] = [];

    // Note: Main extension settings are removed - all settings now only appear in Custom Layers
    // This ensures Service Status and Collision Objects sections only show up under Custom Layers

    // Custom layer entries (if config.layers is available)
    for (const [instanceId, layerConfig] of Object.entries(this.renderer.config.layers)) {
      if (layerConfig?.layerId === LAYER_ID) {
        const config = layerConfig as Partial<LayerSettingsPlanningScene>;

        const fields: SettingsTreeFields = {
          topic: {
            label: t("threeDee:topic"),
            input: "autocomplete",
            value: config.topic ?? DEFAULT_CUSTOM_SETTINGS.topic,
            items: filterMap(this.renderer.topics ?? [], (_topic) =>
              PLANNING_SCENE_DATATYPES.has(_topic.schemaName) ? _topic.name : undefined,
            ),
            help: t("threeDee:planningSceneTopic"),
          },
          defaultColor: {
            label: t("threeDee:defaultColor"),
            input: "rgb",
            value: config.defaultColor ?? DEFAULT_CUSTOM_SETTINGS.defaultColor,
            help: t("threeDee:defaultColorHelp"),
          },
          sceneOpacity: {
            label: t("threeDee:sceneOpacity"),
            input: "number",
            min: 0,
            max: 1,
            step: 0.1,
            precision: 1,
            value: config.sceneOpacity ?? DEFAULT_CUSTOM_SETTINGS.sceneOpacity,
            help: t("threeDee:sceneOpacityHelp"),
          },
          showCollisionObjects: {
            label: t("threeDee:showCollisionObjects"),
            input: "boolean",
            value: config.showCollisionObjects ?? DEFAULT_CUSTOM_SETTINGS.showCollisionObjects,
          },
          showAttachedObjects: {
            label: t("threeDee:showAttachedObjects"),
            input: "boolean",
            value: config.showAttachedObjects ?? DEFAULT_CUSTOM_SETTINGS.showAttachedObjects,
          },
          showOctomap: {
            label: t("threeDee:showOctomap"),
            input: "boolean",
            value: config.showOctomap ?? DEFAULT_CUSTOM_SETTINGS.showOctomap,
            help: t("threeDee:octomapNotImplementedHelp"),
          },
        };

        // Add Service Status and Collision Objects sections to each custom layer
        const layerPath = ["layers", instanceId];
        const serviceError = this.renderer.settings.errors.errors.errorAtPath([
          ...layerPath,
          "service",
        ]);
        const messageError = this.renderer.settings.errors.errors.errorAtPath([
          ...layerPath,
          "messageProcessing",
        ]);

        const children: SettingsTreeChildren = {};

        // Add service status information for this layer
        children.service = {
          label: t("threeDee:serviceStatus"),
          fields: {
            serviceName: {
              label: t("threeDee:service"),
              input: "string",
              readonly: true,
              value: DEFAULT_PLANNING_SCENE_SERVICE,
              help: t("threeDee:serviceHelp"),
            },
            topic: {
              label: t("threeDee:topic"),
              input: "string",
              readonly: true,
              value: config.topic ?? DEFAULT_CUSTOM_SETTINGS.topic,
              help: t("threeDee:planningSceneTopic"),
            },
            status: {
              label: t("threeDee:initialScene"),
              input: "string",
              readonly: true,
              value: this.initialSceneFetched
                ? t("threeDee:loaded")
                : this.fetchingInitialScene
                  ? t("threeDee:loading")
                  : t("threeDee:notLoaded"),
              help: t("threeDee:initialSceneHelp"),
            },
          },
          actions: [
            {
              type: "action" as const,
              id: "refetchService",
              label: t("threeDee:refetch"),
            },
          ],
          error: serviceError,
        };

        // Add collision objects section for this layer
        const collisionChildren = this.getCollisionObjectNodes(instanceId);
        const collisionCount = Object.keys(collisionChildren).length;
        if (collisionCount > 0) {
          children.collisionObjects = {
            label: `${t("threeDee:showCollisionObjects")} (${collisionCount})`,
            children: collisionChildren,
            defaultExpansionState: "collapsed" as const,
            actions: [
              { id: "show-all", type: "action", label: t("threeDee:showAll") },
              { id: "hide-all", type: "action", label: t("threeDee:hideAll") },
            ],
          };
        }

        entries.push({
          path: layerPath,
          node: {
            label: config.label ?? t("threeDee:planningScene"),
            icon: "Cube",
            fields,
            visible: config.visible ?? DEFAULT_CUSTOM_SETTINGS.visible,
            actions: [
              { type: "action", id: "refetch", label: t("threeDee:refetch") },
              { type: "action", id: "delete", label: t("threeDee:delete") },
            ],
            order: layerConfig.order,
            handler: this.#handleLayerSettingsAction,
            children,
            error: messageError,
          },
        });
      }
    }

    return entries;
  }

  private getCollisionObjectNodes(instanceId: string): SettingsTreeChildren {
    const nodes: SettingsTreeChildren = {};

    // Collect IDs from renderables belonging to this layer instance
    const ids = new Set<string>();
    for (const [objectId, renderable] of this.renderables.entries()) {
      const path = renderable.userData.settingsPath;
      if (path[1] === instanceId) {
        ids.add(objectId);
      }
    }

    // Build nodes for each ID that has a renderable
    for (const objectId of ids) {
      const renderable = this.renderables.get(objectId);
      if (renderable && renderable.userData.settingsPath[1] === instanceId) {
        const userData = renderable.userData;
        const settings = userData.settings;
        const collisionObject = userData.collisionObject;

        const primitiveCount = collisionObject.primitives.length;
        const meshCount = collisionObject.meshes.length;
        const planeCount = collisionObject.planes.length;

        const shapeInfo: string[] = [];
        if (primitiveCount > 0) {
          shapeInfo.push(`${primitiveCount} primitive${primitiveCount !== 1 ? "s" : ""}`);
        }
        if (meshCount > 0) {
          shapeInfo.push(`${meshCount} mesh${meshCount !== 1 ? "es" : ""}`);
        }
        if (planeCount > 0) {
          shapeInfo.push(`${planeCount} plane${planeCount !== 1 ? "s" : ""}`);
        }

        const fields: SettingsTreeFields = {
          frameId: {
            label: t("threeDee:frameId"),
            input: "string",
            readonly: true,
            value: userData.frameId,
          },
        };

        // Get errors for this object
        const errors = this.renderer.settings.errors.errors.errorAtPath(userData.settingsPath);

        const node: SettingsTreeNodeWithActionHandler = {
          label: objectId,
          fields,
          visible: settings.visible,
          error: errors,
          handler: this.#handleLayerSettingsAction,
          defaultExpansionState: "collapsed",
          children: (() => {
            const children: SettingsTreeChildren = {};
            // Resolve layer config for saved per-shape visibility
            const layerConfig = this.renderer.config.layers[instanceId] as
              | Partial<LayerSettingsPlanningScene>
              | undefined;

            // Primitives (include type in label)
            for (let i = 0; i < primitiveCount; i++) {
              const key = `primitive_${i}`;
              const savedVisible =
                layerConfig?.collisionObjects?.[objectId]?.shapes?.[key]?.visible;
              const fallbackVisible = userData.shapes.get(key)?.userData.settings.visible ?? true;
              const prim = collisionObject.primitives[i];
              const typeName =
                prim &&
                typeof prim.type === "number" &&
                Object.values(SolidPrimitiveType).includes(prim.type)
                  ? SolidPrimitiveType[prim.type]
                  : "UNKNOWN";
              children[key] = {
                label: `Primitive ${i} (${typeName})`,
                visible: savedVisible ?? fallbackVisible,
                error: this.renderer.settings.errors.errors.errorAtPath([
                  ...userData.settingsPath,
                  key,
                ]),
                defaultExpansionState: "collapsed",
                children: { dummy: undefined }, // Force dropdown arrow
              };
            }
            // Meshes
            for (let i = 0; i < meshCount; i++) {
              const key = `mesh_${i}`;
              const savedVisible =
                layerConfig?.collisionObjects?.[objectId]?.shapes?.[key]?.visible;
              const fallbackVisible = userData.shapes.get(key)?.userData.settings.visible ?? true;
              children[key] = {
                label: `Mesh ${i}`,
                visible: savedVisible ?? fallbackVisible,
                error: this.renderer.settings.errors.errors.errorAtPath([
                  ...userData.settingsPath,
                  key,
                ]),
                defaultExpansionState: "collapsed",
                children: { dummy: undefined }, // Force dropdown arrow
              };
            }
            // Planes
            for (let i = 0; i < planeCount; i++) {
              const key = `plane_${i}`;
              const savedVisible =
                layerConfig?.collisionObjects?.[objectId]?.shapes?.[key]?.visible;
              const fallbackVisible = userData.shapes.get(key)?.userData.settings.visible ?? true;
              children[key] = {
                label: `Plane ${i}`,
                visible: savedVisible ?? fallbackVisible,
                error: this.renderer.settings.errors.errors.errorAtPath([
                  ...userData.settingsPath,
                  key,
                ]),
                defaultExpansionState: "collapsed",
                children: { dummy: undefined }, // Force dropdown arrow
              };
            }
            return children;
          })(),
        };
        nodes[objectId] = node;
      } else {
        // Error-only node for an object that failed to render
        const error = this.renderer.settings.errors.errors.errorAtPath([
          "layers",
          instanceId,
          "collisionObjects",
          objectId,
        ]);
        // Use persisted visibility from config if available so eye toggle sticks
        const layerConfig = this.renderer.config.layers[instanceId] as
          | Partial<LayerSettingsPlanningScene>
          | undefined;
        const savedVisible = layerConfig?.collisionObjects?.[objectId]?.visible;
        const node: SettingsTreeNodeWithActionHandler = {
          label: objectId,
          fields: {},
          visible: savedVisible ?? true,
          error,
          handler: this.#handleLayerSettingsAction,
          defaultExpansionState: "collapsed",
          children: { dummy: undefined }, // Force dropdown arrow
        };
        nodes[objectId] = node;
      }
    }

    return nodes;
  }

  // Note: Main settings action handler removed since all settings are now in Custom Layers

  // Note: handleExtensionSettingsUpdate method removed since all settings are now in Custom Layers

  private handleCollisionObjectSettingsUpdate(
    objectId: string,
    input: string,
    value: unknown,
  ): void {
    const renderable = this.renderables.get(objectId);
    const settings = renderable?.userData.settings;

    // Update the renderable settings
    switch (input) {
      case "visible":
        if (settings) {
          settings.visible = value as boolean;
        }
        // Recalculate visibility based on all layers (will be done in next frame)
        // Don't set renderable.visible directly here, let startFrame() handle it
        break;
    }

    // Save the setting to the configuration
    this.saveCollisionObjectSetting(objectId, input, value);
  }

  // Helper method to save collision object settings to the configuration
  private saveCollisionObjectSetting(objectId: string, input: string, value: unknown): void {
    // Find the instance ID for this collision object
    const instanceId = this.findInstanceIdForCollisionObject(objectId);
    if (!instanceId) {
      return; // Can't save without knowing which layer instance this belongs to
    }

    // Save to configuration
    this.renderer.updateConfig((draft) => {
      // Layers structure is always defined in config

      // CRITICAL: Do not create a new layer if it doesn't exist!
      // This was causing layer duplication
      if (!draft.layers[instanceId]) {
        log.error(
          `Layer ${instanceId} does not exist in config! Cannot save collision object setting.`,
        );
        return; // Exit early to prevent creating duplicate layers
      }

      const layerConfig = draft.layers[instanceId] as LayerSettingsPlanningScene;

      // Ensure collisionObjects structure exists
      if (!layerConfig.collisionObjects) {
        layerConfig.collisionObjects = {};
      }
      if (!layerConfig.collisionObjects[objectId]) {
        layerConfig.collisionObjects[objectId] = {} as CollisionObjectLayerConfig;
      }

      // Save the specific setting with proper typing
      const objConfig = layerConfig.collisionObjects[objectId]!;
      if (input === "visible") {
        objConfig.visible = value as boolean;
      } else if (input === "color") {
        objConfig.color = value as string;
      } else if (input === "opacity") {
        objConfig.opacity = value as number;
      }
    });

    // Update the settings tree to reflect the changes
    this.updateSettingsTree();
  }

  // Helper method to find which instance a collision object belongs to
  private findInstanceIdForCollisionObject(objectId: string): string | undefined {
    // Method 1: Check if we have a current instance ID from message processing
    if (this.currentInstanceId) {
      // Verify this instance actually exists in the config
      if (this.renderer.config.layers[this.currentInstanceId]?.layerId === LAYER_ID) {
        return this.currentInstanceId;
      }
    }

    // Method 2: Get instance ID from the renderable's settings path
    const renderable = this.renderables.get(objectId);
    if (renderable?.userData.settingsPath && renderable.userData.settingsPath.length >= 2) {
      const instanceId = renderable.userData.settingsPath[1]!;
      // Verify this instance exists in the config
      if (this.renderer.config.layers[instanceId]?.layerId === LAYER_ID) {
        return instanceId;
      }
    }

    // Method 3: Find the first available planning scene layer
    for (const [instanceId, layerConfig] of Object.entries(this.renderer.config.layers)) {
      if (layerConfig?.layerId === LAYER_ID) {
        return instanceId;
      }
    }

    return undefined;
  }

  public override getSubscriptions(): readonly AnyRendererSubscription[] {
    return [
      {
        type: "schema",
        schemaNames: PLANNING_SCENE_DATATYPES,
        subscription: {
          shouldSubscribe: this.#shouldSubscribe,
          handler: this.handlePlanningSceneMessage.bind(this),
          filterQueue: undefined,
        },
      },
    ];
  }

  #shouldSubscribe = (topic: string): boolean => {
    // Subscribe to topics that are configured in custom layers and have the correct schema
    for (const layerConfig of Object.values(this.renderer.config.layers)) {
      if (layerConfig?.layerId === LAYER_ID) {
        const config = layerConfig as Partial<LayerSettingsPlanningScene>;
        if (config.topic === topic && config.visible === true) {
          // Verify the topic has the correct schema
          const topicInfo = this.renderer.topicsByName?.get(topic);
          if (topicInfo && PLANNING_SCENE_DATATYPES.has(topicInfo.schemaName)) {
            return true;
          }
        }
      }
    }
    return false;
  };

  // Override startFrame to update each collision object individually (Direct Child Manipulation)
  public override startFrame(
    currentTime: bigint,
    renderFrameId: AnyFrameId,
    fixedFrameId: AnyFrameId,
  ): void {
    // Fetch initial scene if we haven't done so yet and we're not already fetching
    // Only fetch if there are visible planning scene layers configured
    const hasVisibleLayers = Object.values(this.renderer.config.layers).some(
      (layer) => layer?.layerId === LAYER_ID && layer.visible === true,
    );

    if (!this.initialSceneFetched && !this.fetchingInitialScene && hasVisibleLayers) {
      void this.fetchInitialScene();
    } else if (!this.initialSceneFetched && !this.fetchingInitialScene && !hasVisibleLayers) {
      // Clear any existing renderables if there are no visible layers
      if (this.renderables.size > 0) {
        this.removeAllRenderables();
        this.currentScene = undefined;
        this.initialSceneFetched = false;
      }
    }

    // Call the base class implementation to handle standard visibility and transforms
    super.startFrame(currentTime, renderFrameId, fixedFrameId);

    // Apply additional PlanningScene-specific visibility logic
    for (const collisionObject of this.renderables.values()) {
      // Skip if already invisible from base class processing
      if (!collisionObject.visible) {
        continue;
      }

      // Check if any planning scene layer is visible and shows collision objects
      let layerAllowsVisibility = false;
      for (const layerConfig of Object.values(this.renderer.config.layers)) {
        if (layerConfig?.layerId === LAYER_ID) {
          const config = layerConfig as Partial<LayerSettingsPlanningScene>;
          const layerVisible = config.visible !== false;
          const showCollisionObjects = config.showCollisionObjects !== false;
          if (layerVisible && showCollisionObjects) {
            layerAllowsVisibility = true;
            break; // Found at least one layer that allows visibility
          }
        }
      }

      // Apply layer-level visibility override
      if (!layerAllowsVisibility) {
        collisionObject.visible = false;
        continue;
      }

      if (collisionObject.userData.isAttachedObject) {
        collisionObject.visible =
          this.settings.showAttachedObjects && collisionObject.userData.settings.visible;
      } else {
        collisionObject.visible =
          this.settings.showCollisionObjects && collisionObject.userData.settings.visible;
      }
    }
  }

  // Handle planning scene messages (differential updates)
  private handlePlanningSceneMessage = (messageEvent: PartialMessageEvent<PlanningScene>): void => {
    const scene = messageEvent.message;
    const topic = messageEvent.topic;

    // Check if there are any planning scene layers configured at all
    const hasConfiguredLayers = Object.values(this.renderer.config.layers).some(
      (layer) => layer?.layerId === LAYER_ID,
    );

    if (!hasConfiguredLayers) {
      // No planning scene layers configured, ignore the message
      return;
    }

    // Update message context for creating renderables
    this.topic = topic;
    this.receiveTime = toNanoSec(messageEvent.receiveTime);
    this.messageTime = messageEvent.message.robot_state?.joint_state?.header?.stamp
      ? BigInt(messageEvent.message.robot_state.joint_state.header.stamp.sec ?? 0) * 1000000000n +
        BigInt(messageEvent.message.robot_state.joint_state.header.stamp.nsec ?? 0)
      : toNanoSec(messageEvent.receiveTime);

    // Find the instance ID for this topic
    this.currentInstanceId = this.findInstanceIdForTopic(topic);

    // Load settings from the layer configuration for this topic
    this.settings = this.loadSettingsFromConfig(this.currentInstanceId);

    // Validate message structure
    if (!this.validatePlanningSceneMessage(scene, topic)) {
      return;
    }

    try {
      // Clear any previous message processing errors
      this.renderer.settings.errors.remove(
        ["extensions", PlanningSceneExtension.extensionId, "messageProcessing"],
        MESSAGE_PROCESSING_ERROR,
      );

      if (scene.is_diff === true) {
        // Apply differential update to existing scene
        this.applyDifferentialUpdate(scene);
      } else {
        // Replace entire scene
        this.replaceEntireScene(scene);
      }
    } catch (error) {
      const errorMessage = `Failed to process planning scene message from topic "${topic}": ${
        error instanceof Error ? error.message : String(error)
      }`;

      // Report error to settings tree
      this.renderer.settings.errors.add(
        ["extensions", PlanningSceneExtension.extensionId, "messageProcessing"],
        MESSAGE_PROCESSING_ERROR,
        errorMessage,
      );
    }
  };

  // Fetch initial scene via service call
  private async fetchInitialScene(): Promise<void> {
    // Check if service client is available
    if (!this.serviceClient) {
      const errorMessage = t("threeDee:serviceClientUnavailable");

      // Report error to settings tree
      this.renderer.settings.errors.add(
        ["extensions", PlanningSceneExtension.extensionId, "service"],
        SERVICE_ERROR,
        errorMessage,
      );
      return;
    }

    // Prevent multiple concurrent service calls
    if (this.fetchingInitialScene) {
      return;
    }

    this.fetchingInitialScene = true;

    // Try to recover settings
    this.currentInstanceId = this.findFirstPlanningSceneInstanceId();
    if (this.currentInstanceId) {
      this.settings = this.loadSettingsFromConfig(this.currentInstanceId);
    }

    try {
      // Clear any previous service errors
      this.renderer.settings.errors.remove(
        ["extensions", PlanningSceneExtension.extensionId, "service"],
        SERVICE_ERROR,
      );

      const request: GetPlanningSceneRequest = {
        components: createMinimalPlanningSceneComponents(),
      };

      log.info(`Fetching initial planning scene from service: ${DEFAULT_PLANNING_SCENE_SERVICE}`);

      // Call the service directly

      const serviceCallPromise = this.serviceClient(DEFAULT_PLANNING_SCENE_SERVICE, request);

      const response = (await serviceCallPromise) as GetPlanningSceneResponse | undefined;

      // Validate the response structure with detailed error messages
      if (response == undefined) {
        throw new Error(
          `Service '${DEFAULT_PLANNING_SCENE_SERVICE}' returned null or undefined response`,
        );
      }

      if (typeof response !== "object") {
        throw new Error(
          `Service '${DEFAULT_PLANNING_SCENE_SERVICE}' returned invalid response type: ${typeof response} (expected object)`,
        );
      }

      if (!Object.prototype.hasOwnProperty.call(response, "scene")) {
        throw new Error(
          `Service '${DEFAULT_PLANNING_SCENE_SERVICE}' response missing required 'scene' field. Available fields: ${Object.keys(response).join(", ")}`,
        );
      }

      const scene = response.scene as PartialMessage<PlanningScene> | undefined;

      if (scene == undefined) {
        throw new Error(
          `Service '${DEFAULT_PLANNING_SCENE_SERVICE}' returned null or undefined scene data`,
        );
      }

      // Validate the scene structure
      const isValid = this.validatePlanningSceneMessage(scene, DEFAULT_PLANNING_SCENE_SERVICE);
      if (!isValid) {
        throw new Error(t("threeDee:invalidPlanningSceneData"));
      }

      log.info("Successfully fetched initial planning scene from service");

      // Store the complete scene as baseline for differential updates
      this.currentScene = scene;

      // Process the initial scene (treat as full scene replacement)
      try {
        this.replaceEntireScene(scene);
      } catch (error) {
        // This try catch is a just-in-case for if replaceEntireScene fails to handle an internal error.
        log.error(`Failed to replace entire scene:`, error);
      }

      // Mark as successfully fetched
      this.initialSceneFetched = true;

      // Clear any service errors on success
      this.renderer.settings.errors.remove(
        ["extensions", PlanningSceneExtension.extensionId, "service"],
        SERVICE_ERROR,
      );
    } catch (error) {
      let errorMessage: string;

      if (error instanceof Error) {
        // Check for specific error types and provide helpful guidance
        if (error.message.includes("Service client")) {
          errorMessage = `Service client error: ${error.message}`;
        } else if (error.message.includes("not found") || error.message.includes("unavailable")) {
          errorMessage = `Service unavailable: The '${DEFAULT_PLANNING_SCENE_SERVICE}' service is not available. Make sure MoveIt is running and the planning scene service is advertised.`;
        } else {
          errorMessage = `Failed to fetch initial planning scene from '${DEFAULT_PLANNING_SCENE_SERVICE}': ${error.message}`;
        }
      } else {
        errorMessage = `Failed to fetch initial planning scene from '${DEFAULT_PLANNING_SCENE_SERVICE}': ${String(error)}`;
      }

      log.warn(errorMessage);

      // Report error to settings tree
      this.renderer.settings.errors.add(
        ["extensions", PlanningSceneExtension.extensionId, "service"],
        SERVICE_ERROR,
        errorMessage,
      );

      // Don't mark as fetched so we can retry later
      this.initialSceneFetched = false;
    } finally {
      this.fetchingInitialScene = false;

      // Update settings tree to reflect the new status
      this.updateSettingsTree();
    }
  }

  // Apply differential update to existing scene
  private applyDifferentialUpdate(scene: PartialMessage<PlanningScene>): void {
    if (!this.currentScene) {
      // If we don't have a base scene, try to fetch it first
      log.warn(
        "Received differential update without base scene, attempting to fetch initial scene",
      );

      if (!this.fetchingInitialScene && !this.initialSceneFetched) {
        void this.fetchInitialScene();
      } else {
        this.replaceEntireScene(scene);
      }

      return;
    }

    // Update current scene by merging the differential data first
    // This ensures that color information and other data is available during processing
    this.currentScene = this.mergeSceneData(this.currentScene, scene);

    // Process collision objects if present
    if (scene.world?.collision_objects) {
      for (const collisionObject of scene.world.collision_objects) {
        if (collisionObject) {
          this.applyCollisionObjectOperation(collisionObject);
        }
      }
    }

    // Handle color-only updates: if object_colors changed, update existing objects that aren't in collision_objects
    if (scene.object_colors && scene.object_colors.length > 0) {
      const processedObjectIds = new Set(
        scene.world?.collision_objects?.map((obj) => obj?.id).filter(Boolean) ?? [],
      );

      for (const objectColor of scene.object_colors) {
        if (!objectColor?.id) {
          continue;
        }

        // Only update existing objects that weren't already processed above
        if (!processedObjectIds.has(objectColor.id) && this.renderables.has(objectColor.id)) {
          const existingRenderable = this.renderables.get(objectColor.id);
          if (existingRenderable) {
            // Update the color in userData and trigger visual update
            const sceneColor = this.getObjectColorFromScene(objectColor.id);
            if (sceneColor) {
              existingRenderable.userData.settings.color = sceneColor.color;
              existingRenderable.userData.settings.opacity =
                this.settings.sceneOpacity * sceneColor.opacity;
              existingRenderable.update(existingRenderable.userData.collisionObject);
            }
          }
        }
      }
    }

    // Process robot state updates if present
    if (scene.robot_state) {
      this.processRobotState(scene.robot_state, scene);
    }
  }

  // Process robot state including attached collision objects
  private processRobotState(
    robotState: PartialMessage<RobotState>,
    _scene?: PartialMessage<PlanningScene>,
  ): void {
    // Process attached collision objects
    if (robotState.attached_collision_objects) {
      for (const attachedObject of robotState.attached_collision_objects) {
        if (attachedObject?.object && attachedObject.link_name) {
          this.processAttachedCollisionObject(attachedObject);
        }
      }
    }
  }

  // Process an attached collision object
  private processAttachedCollisionObject(
    attachedObject: PartialMessage<AttachedCollisionObject>,
  ): void {
    const object = attachedObject.object!;
    const linkName = attachedObject.link_name!;
    const objectId = object.id!;

    // Create a modified collision object with the link frame
    const attachedCollisionObject: PartialMessage<CollisionObject> = {
      ...object,
      header: {
        ...object.header,
        frame_id: linkName, // Use the link name as the frame instead of the object's original frame
      },
    };

    // Apply the collision object operation (usually ADD for attached objects)
    this.applyCollisionObjectOperation(attachedCollisionObject, true);

    log.info(`Processed attached collision object '${objectId}' attached to link '${linkName}'`);
  }

  // Replace entire scene
  private replaceEntireScene(scene: PartialMessage<PlanningScene>): void {
    // Clear all existing renderables
    this.removeAllRenderables();

    // Store the new scene
    this.currentScene = scene;

    // Process collision objects if present
    if (scene.world?.collision_objects) {
      for (const collisionObject of scene.world.collision_objects) {
        if (collisionObject) {
          this.applyCollisionObjectOperation(collisionObject);
        }
      }
    }

    // Process robot state if present
    if (scene.robot_state) {
      this.processRobotState(scene.robot_state, scene);
    }
  }

  // Merge differential scene data into existing scene
  private mergeSceneData(
    baseScene: PartialMessage<PlanningScene>,
    diffScene: PartialMessage<PlanningScene>,
  ): PartialMessage<PlanningScene> {
    // Create a deep copy of the base scene
    const mergedScene: PartialMessage<PlanningScene> = {
      ...baseScene,
      ...diffScene,
    };

    // Handle world data merging
    if (baseScene.world && diffScene.world) {
      mergedScene.world = {
        ...baseScene.world,
        ...diffScene.world,
      };

      // For collision objects, we need special handling since they can be added/removed/modified
      if (diffScene.world.collision_objects && diffScene.world.collision_objects.length > 0) {
        // The collision objects in the diff will be processed by applyCollisionObjectOperation
        // which will handle ADD/REMOVE/APPEND/MOVE operations appropriately
        // For now, we keep the base collision objects and let the operation handler manage changes
        mergedScene.world.collision_objects = baseScene.world.collision_objects ?? [];
      }
    }

    // Handle robot state merging
    if (baseScene.robot_state && diffScene.robot_state) {
      mergedScene.robot_state = {
        ...baseScene.robot_state,
        ...diffScene.robot_state,
      };
    }

    // Handle object_colors merging - merge colors instead of replacing the entire array
    if (baseScene.object_colors || diffScene.object_colors) {
      const baseColors = baseScene.object_colors ?? [];
      const diffColors = diffScene.object_colors ?? [];

      // Create a map of existing colors by object ID for efficient lookup
      const colorMap = new Map<string, (typeof baseColors)[0]>();

      // Add base colors to the map
      for (const colorEntry of baseColors) {
        if (colorEntry?.id) {
          colorMap.set(colorEntry.id, colorEntry);
        }
      }

      // Add/update with diff colors (this will overwrite existing colors for the same object ID)
      for (const colorEntry of diffColors) {
        if (colorEntry?.id) {
          colorMap.set(colorEntry.id, colorEntry);
        }
      }

      // Convert back to array
      mergedScene.object_colors = Array.from(colorMap.values());
    }

    return mergedScene;
  }

  // Validate planning scene message structure
  private validatePlanningSceneMessage(
    scene: PartialMessage<PlanningScene>,
    topic: string,
  ): boolean {
    try {
      // Since scene is a PartialMessage, it should always be defined, but we can check for empty object
      if (Object.keys(scene).length === 0) {
        throw new Error(`Received empty planning scene message from topic "${topic}"`);
      }

      // Check for required fields based on message type
      if (scene.is_diff === true) {
        // For differential updates, we need at least some content to apply
        const hasCollisionObjects =
          scene.world?.collision_objects != undefined && scene.world.collision_objects.length > 0;
        const hasRobotState = scene.robot_state != undefined;
        const hasTransforms =
          scene.fixed_frame_transforms != undefined && scene.fixed_frame_transforms.length > 0;

        if (!hasCollisionObjects && !hasRobotState && !hasTransforms) {
          throw new Error(
            `Received differential planning scene message with no content from topic "${topic}"`,
          );
        }

        // Validate collision objects in differential updates
        if (hasCollisionObjects) {
          this.validateCollisionObjects(
            scene.world!.collision_objects as (PartialMessage<CollisionObject> | undefined)[],
            topic,
          );
        }
      } else {
        // For full scene updates, we expect a world object (even if empty)
        if (scene.world == undefined) {
          throw new Error(
            `Received full planning scene message without world data from topic "${topic}"`,
          );
        }

        // Validate collision objects in full scene updates
        if (scene.world.collision_objects && scene.world.collision_objects.length > 0) {
          this.validateCollisionObjects(
            scene.world.collision_objects as (PartialMessage<CollisionObject> | undefined)[],
            topic,
          );
        }
      }

      return true;
    } catch (error) {
      const errorMessage = `Message validation failed in topic "${topic}": ${error instanceof Error ? error.message : String(error)}`;

      // Report validation error to settings tree
      this.renderer.settings.errors.add(
        ["extensions", PlanningSceneExtension.extensionId, "messageProcessing"],
        MESSAGE_PROCESSING_ERROR,
        errorMessage,
      );

      return false;
    }
  }

  // Validate collision objects structure
  private validateCollisionObjects(
    collisionObjects: (PartialMessage<CollisionObject> | undefined)[],
    _topic: string,
  ): void {
    for (let i = 0; i < collisionObjects.length; i++) {
      const obj = collisionObjects[i];
      if (obj == undefined) {
        throw new Error(`collision_object[${i}] is null or undefined`);
      }

      if (obj.id == undefined || typeof obj.id !== "string") {
        throw new Error(`collision_object[${i}] missing id`);
      }

      if (obj.operation == undefined || typeof obj.operation !== "number") {
        throw new Error(`Collision object '${obj.id}' has invalid or missing 'operation' field`);
      }

      // Validate operation-specific requirements
      if (
        obj.operation === CollisionObjectOperation.ADD ||
        obj.operation === CollisionObjectOperation.MOVE
      ) {
        if (obj.header?.frame_id == undefined) {
          throw new Error(
            `Collision object '${obj.id}' with ${CollisionObjectOperation[obj.operation]} operation requires header.frame_id`,
          );
        }

        if (obj.pose == undefined) {
          throw new Error(
            `Collision object '${obj.id}' with ${CollisionObjectOperation[obj.operation]} operation requires pose`,
          );
        }
      }

      // Validate geometry data
      if (obj.primitives != undefined) {
        this.validatePrimitives(obj.primitives, obj.id);
      }

      if (obj.meshes != undefined) {
        this.validateMeshes(obj.meshes, obj.id);
      }
    }
  }

  // Validate primitive shapes
  private validatePrimitives(
    primitives: (PartialMessage<SolidPrimitive> | undefined)[],
    _objectId: string,
  ): void {
    for (let i = 0; i < primitives.length; i++) {
      const primitive = primitives[i];
      if (primitive == undefined) {
        throw new Error(`primitive[${i}] is null or undefined`);
      }

      if (primitive.type == undefined || typeof primitive.type !== "number") {
        throw new Error(`primitive[${i}] missing type`);
      }

      if (primitive.dimensions == undefined || !Array.isArray(primitive.dimensions)) {
        throw new Error(`primitive[${i}] missing dimensions`);
      }

      // Validate dimensions based on primitive type
      switch (primitive.type) {
        case SolidPrimitiveType.BOX:
          if (primitive.dimensions.length < 3) {
            throw new Error(`primitive[${i}] BOX requires 3 dimensions [x, y, z]`);
          }
          break;
        case SolidPrimitiveType.SPHERE:
          if (primitive.dimensions.length < 1) {
            throw new Error(`primitive[${i}] SPHERE requires 1 dimension [radius]`);
          }
          break;
        case SolidPrimitiveType.CYLINDER:
        case SolidPrimitiveType.CONE:
          if (primitive.dimensions.length < 2) {
            throw new Error(
              `primitive[${i}] ${primitive.type === SolidPrimitiveType.CYLINDER ? "CYLINDER" : "CONE"} requires 2 dimensions [height, radius]`,
            );
          }
          break;
      }

      // Validate dimension values are positive
      for (let j = 0; j < primitive.dimensions.length; j++) {
        const dim = primitive.dimensions[j];
        if (typeof dim !== "number" || dim <= 0) {
          throw new Error(`primitive[${i}] invalid dimensions[${j}]: ${dim} (> 0)`);
        }
      }
    }
  }

  // Validate mesh data
  private validateMeshes(meshes: (PartialMessage<Mesh> | undefined)[], _objectId: string): void {
    for (let i = 0; i < meshes.length; i++) {
      const mesh = meshes[i];
      if (mesh == undefined) {
        throw new Error(`mesh[${i}] is null or undefined`);
      }

      if (mesh.vertices == undefined || !Array.isArray(mesh.vertices)) {
        throw new Error(`mesh[${i}] missing vertices array`);
      }

      if (mesh.triangles == undefined || !Array.isArray(mesh.triangles)) {
        throw new Error(`mesh[${i}] missing triangles array`);
      }

      if (mesh.vertices.length === 0) {
        throw new Error(`mesh[${i}] has no vertices`);
      }

      if (mesh.triangles.length === 0) {
        throw new Error(`mesh[${i}] has no triangles`);
      }

      // Validate triangle indices
      for (let j = 0; j < mesh.triangles.length; j++) {
        const triangle = mesh.triangles[j];
        if (triangle?.vertex_indices == undefined || triangle.vertex_indices.length !== 3) {
          throw new Error(`mesh[${i}] triangle[${j}] needs 3 vertex indices`);
        }

        for (let k = 0; k < 3; k++) {
          const vertexIndex = triangle.vertex_indices[k];
          if (
            typeof vertexIndex !== "number" ||
            vertexIndex < 0 ||
            vertexIndex >= mesh.vertices.length
          ) {
            throw new Error(
              `mesh[${i}] triangle[${j}] invalid vertex index ${vertexIndex} (0-${mesh.vertices.length - 1})`,
            );
          }
        }
      }
    }
  }

  // Apply collision object operations (ADD, REMOVE, APPEND, MOVE)
  private applyCollisionObjectOperation(
    object: PartialMessage<CollisionObject>,
    // eslint-disable-next-line @lichtblick/no-boolean-parameters
    isAttachedObject: boolean = false,
  ): void {
    // Validate required fields
    if (object.id == undefined) {
      const errorMessage = t("threeDee:collisionObjectMissingId");

      // Report error to settings tree (use layer path since we don't have an ID)
      const instId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
      if (instId) {
        this.renderer.settings.errors.add(
          ["layers", instId, "messageProcessing"],
          MESSAGE_PROCESSING_ERROR,
          errorMessage,
        );
      }
      return;
    }

    if (object.operation == undefined) {
      const errorMessage = `Collision object '${object.id}' missing required 'operation' field, cannot process`;

      // Report error to settings tree on the layer path
      const instId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
      if (instId) {
        this.renderer.settings.errors.add(
          ["layers", instId, "collisionObjects", object.id],
          MESSAGE_PROCESSING_ERROR,
          errorMessage,
        );
      }
      return;
    }

    const objectId = object.id;
    const operation = object.operation;

    try {
      // Differential update - only process changed objects for performance optimization
      if (
        operation === CollisionObjectOperation.ADD ||
        operation === CollisionObjectOperation.APPEND ||
        operation === CollisionObjectOperation.MOVE
      ) {
        const fullObject = this.ensureFullCollisionObject(object); // This throws if invalid
        if (!this.hasObjectChanged(fullObject)) {
          // Object hasn't changed, skip processing
          return;
        }
      }

      // Clear any previous errors for this collision object
      const errInstanceId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
      if (errInstanceId) {
        this.renderer.settings.errors.remove(
          ["layers", errInstanceId, "collisionObjects", objectId],
          MESSAGE_PROCESSING_ERROR,
        );
      }

      switch (operation) {
        case CollisionObjectOperation.ADD: {
          this.handleAddOperation(object, isAttachedObject);
          break;
        }
        case CollisionObjectOperation.REMOVE: {
          this.handleRemoveOperation(objectId);
          break;
        }
        case CollisionObjectOperation.APPEND: {
          this.handleAppendOperation(object);
          break;
        }
        case CollisionObjectOperation.MOVE: {
          this.handleMoveOperation(object);
          break;
        }
        default: {
          const errorMessage = `Unknown collision object operation: ${operation} (${CollisionObjectOperation[operation] ?? "UNKNOWN"}). Valid operations are: ADD (${CollisionObjectOperation.ADD}), REMOVE (${CollisionObjectOperation.REMOVE}), APPEND (${CollisionObjectOperation.APPEND}), MOVE (${CollisionObjectOperation.MOVE})`;

          // Report error to settings tree
          const instId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
          if (instId) {
            this.renderer.settings.errors.add(
              ["layers", instId, "collisionObjects", objectId],
              MESSAGE_PROCESSING_ERROR,
              errorMessage,
            );
          }
          break;
        }
      }
    } catch (error) {
      const operationName = CollisionObjectOperation[operation] || `UNKNOWN(${operation})`;
      const errorMessage = `Failed to apply ${operationName} operation: ${error instanceof Error ? error.message : String(error)}`;

      // Report error to settings tree
      const instId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
      if (instId) {
        this.renderer.settings.errors.add(
          ["layers", instId, "collisionObjects", objectId],
          MESSAGE_PROCESSING_ERROR,
          errorMessage,
        );
      }
    }
  }

  // Handle ADD operation - creates new CollisionObjectRenderable instances
  private handleAddOperation(
    object: PartialMessage<CollisionObject>,
    // eslint-disable-next-line @lichtblick/no-boolean-parameters
    isAttachedObject: boolean = false,
  ): void {
    const objectId = object.id!;

    // Validate required fields for ADD operation
    if (object.header?.frame_id == undefined) {
      throw new Error(
        `ADD operation for object '${objectId}' requires header.frame_id to specify the coordinate frame`,
      );
    }

    if (object.pose == undefined) {
      throw new Error(
        `ADD operation for object '${objectId}' requires pose to specify position and orientation`,
      );
    }

    try {
      // Validate pose values
      const pose = object.pose;
      if (
        pose.position &&
        ((pose.position.x != undefined && !isFinite(pose.position.x)) ||
          (pose.position.y != undefined && !isFinite(pose.position.y)) ||
          (pose.position.z != undefined && !isFinite(pose.position.z)))
      ) {
        throw new Error(
          `ADD operation for object '${objectId}' has invalid position values: [${pose.position.x ?? "undefined"}, ${pose.position.y ?? "undefined"}, ${pose.position.z ?? "undefined"}]`,
        );
      }

      if (
        pose.orientation &&
        ((pose.orientation.x != undefined && !isFinite(pose.orientation.x)) ||
          (pose.orientation.y != undefined && !isFinite(pose.orientation.y)) ||
          (pose.orientation.z != undefined && !isFinite(pose.orientation.z)) ||
          (pose.orientation.w != undefined && !isFinite(pose.orientation.w)))
      ) {
        throw new Error(
          `ADD operation for object '${objectId}' has invalid orientation values: [${pose.orientation.x ?? "undefined"}, ${pose.orientation.y ?? "undefined"}, ${pose.orientation.z ?? "undefined"}, ${pose.orientation.w ?? "undefined"}]`,
        );
      }

      // Ensure all properties are fully populated
      const fullObject = this.ensureFullCollisionObject(object);

      // Count geometry for logging
      const primitiveCount = fullObject.primitives.length;
      const meshCount = fullObject.meshes.length;
      const planeCount = fullObject.planes.length;
      const totalShapes = primitiveCount + meshCount + planeCount;

      // Remove existing object if it exists (ADD replaces existing objects)
      if (this.renderables.has(objectId)) {
        log.info(`Replacing existing collision object '${objectId}' with ADD operation`);
        this.handleRemoveOperation(objectId);
      }

      // Extract color from planning scene message, falling back to default
      const sceneColor = this.getObjectColorFromScene(objectId);
      const objectColor = sceneColor?.color ?? this.settings.defaultColor;

      // Get saved settings for this collision object from configuration
      const instanceId = this.findInstanceIdForCollisionObject(objectId) ?? this.currentInstanceId;
      const layerConfig = instanceId
        ? (this.renderer.config.layers[instanceId] as
            | Partial<LayerSettingsPlanningScene>
            | undefined)
        : undefined;
      const savedObjectSettings = layerConfig?.collisionObjects?.[objectId];

      // Apply scene opacity multiplicatively: sceneOpacity * messageOpacity (or 1.0 if no message opacity)
      const sceneOpacity = this.settings.sceneOpacity;
      const messageOpacity = sceneColor?.opacity ?? 1.0;
      const finalOpacity = sceneOpacity * messageOpacity;

      // Create user data for the collision object
      const userData: CollisionObjectUserData = {
        topic: this.topic,
        receiveTime: this.receiveTime,
        messageTime: this.messageTime,
        frameId: fullObject.header.frame_id,
        pose: fullObject.pose,
        settingsPath: ["layers", instanceId ?? "unknown", "collisionObjects", objectId],
        settings: {
          visible: savedObjectSettings?.visible ?? true,
          color: savedObjectSettings?.color ?? objectColor, // Use saved color, then scene color, then default
          opacity: finalOpacity, // Use scene opacity * message opacity
        },
        collisionObject: fullObject,
        shapes: new Map<string, Renderable>(),
        isAttachedObject,
      };

      // Create the collision object renderable
      const renderable = new CollisionObjectRenderable(objectId, this.renderer, userData);

      // Update the renderable with the collision object data
      renderable.update(fullObject);

      // Set visibility based on extension and object settings
      const showToggle = isAttachedObject
        ? this.settings.showAttachedObjects
        : this.settings.showCollisionObjects;
      renderable.visible = showToggle && userData.settings.visible;

      // Add to the scene
      this.add(renderable);
      this.renderables.set(objectId, renderable);

      log.info(
        `Added collision object '${objectId}' in frame '${fullObject.header.frame_id}' with ${totalShapes} shapes (${primitiveCount} primitives, ${meshCount} meshes, ${planeCount} planes)`,
      );
    } catch (error) {
      throw new Error(
        `ADD operation for object '${objectId}' failed: ${error instanceof Error ? error.message : String(error)}`,
      );
    }
  }

  // Handle REMOVE operation - disposes and removes renderables
  private handleRemoveOperation(objectId: string): void {
    const renderable = this.renderables.get(objectId);
    if (!renderable) {
      // Not an error - object might not exist or already removed
      log.info(
        `Collision object '${objectId}' not found for REMOVE operation (may already be removed)`,
      );
      return;
    }

    // Remove from scene and dispose resources
    this.remove(renderable);
    renderable.dispose();
    this.renderables.delete(objectId);

    // Clean up object hash when removed for performance optimization
    this.objectHashes.delete(objectId);

    // Clear any errors for this object
    const errInstanceId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
    if (errInstanceId) {
      this.renderer.settings.errors.clearPath([
        "layers",
        errInstanceId,
        "collisionObjects",
        objectId,
      ]);
    }

    log.info(`Removed collision object '${objectId}'`);
  }

  // Handle MOVE operation - updates poses without changing geometry
  private handleMoveOperation(object: PartialMessage<CollisionObject>): void {
    const objectId = object.id!;
    const renderable = this.renderables.get(objectId);

    if (renderable == undefined) {
      throw new Error(
        `Cannot MOVE collision object '${objectId}': object not found. Make sure the object was previously added with an ADD operation.`,
      );
    }

    // Validate that geometry arrays are empty for MOVE operation
    const primitiveCount = object.primitives?.length ?? 0;
    const meshCount = object.meshes?.length ?? 0;
    const planeCount = object.planes?.length ?? 0;
    const hasGeometry = primitiveCount > 0 || meshCount > 0 || planeCount > 0;

    if (hasGeometry) {
      throw new Error(
        `MOVE operation for object '${objectId}' must have empty geometry arrays. Found ${primitiveCount} primitives, ${meshCount} meshes, ${planeCount} planes. Use APPEND operation to add geometry or ADD operation to replace the entire object.`,
      );
    }

    // Validate required fields for MOVE operation
    if (object.header?.frame_id == undefined) {
      throw new Error(
        `MOVE operation for object '${objectId}' requires header.frame_id to specify the target coordinate frame`,
      );
    }

    if (object.pose == undefined) {
      throw new Error(
        `MOVE operation for object '${objectId}' requires pose to specify the new position and orientation`,
      );
    }

    try {
      // Validate pose values
      const pose = object.pose;
      if (
        pose.position &&
        ((pose.position.x != undefined && !isFinite(pose.position.x)) ||
          (pose.position.y != undefined && !isFinite(pose.position.y)) ||
          (pose.position.z != undefined && !isFinite(pose.position.z)))
      ) {
        throw new Error(
          `MOVE operation for object '${objectId}' has invalid position values: [${pose.position.x ?? "undefined"}, ${pose.position.y ?? "undefined"}, ${pose.position.z ?? "undefined"}]`,
        );
      }

      if (
        pose.orientation &&
        ((pose.orientation.x != undefined && !isFinite(pose.orientation.x)) ||
          (pose.orientation.y != undefined && !isFinite(pose.orientation.y)) ||
          (pose.orientation.z != undefined && !isFinite(pose.orientation.z)) ||
          (pose.orientation.w != undefined && !isFinite(pose.orientation.w)))
      ) {
        throw new Error(
          `MOVE operation for object '${objectId}' has invalid orientation values: [${pose.orientation.x ?? "undefined"}, ${pose.orientation.y ?? "undefined"}, ${pose.orientation.z ?? "undefined"}, ${pose.orientation.w ?? "undefined"}]`,
        );
      }

      // Update the pose and frame information without changing geometry
      renderable.userData.pose = this.ensureFullPose(object.pose);
      renderable.userData.frameId = object.header.frame_id!;
      renderable.userData.collisionObject = {
        ...renderable.userData.collisionObject,
        header: this.ensureFullHeader(object.header),
        pose: this.ensureFullPose(object.pose),
      };

      // Update color and opacity from planning scene if available (colors may have changed)
      const sceneColor = this.getObjectColorFromScene(objectId);
      if (sceneColor) {
        renderable.userData.settings.color = sceneColor.color;
        // Apply scene opacity multiplicatively
        const messageOpacity = sceneColor.opacity;
        renderable.userData.settings.opacity = this.settings.sceneOpacity * messageOpacity;
        // Trigger visual update to apply new colors
        renderable.update(renderable.userData.collisionObject);
      }

      log.info(
        `Moved collision object '${objectId}' to new pose in frame '${object.header.frame_id}'`,
      );
    } catch (error) {
      throw new Error(
        `MOVE operation for object '${objectId}' failed: ${error instanceof Error ? error.message : String(error)}`,
      );
    }
  }

  // Handle APPEND operation - adds shapes to existing objects
  private handleAppendOperation(object: PartialMessage<CollisionObject>): void {
    const objectId = object.id!;
    const renderable = this.renderables.get(objectId);

    if (renderable == undefined) {
      throw new Error(
        `Cannot APPEND to collision object '${objectId}': object not found. Make sure the object was previously added with an ADD operation.`,
      );
    }

    // Get the existing collision object data
    const existingObject = renderable.userData.collisionObject;

    // Validate that we have something to append
    const newPrimitives = object.primitives?.filter((p) => p != undefined) ?? [];
    const newPrimitivePoses = object.primitive_poses?.filter((p) => p != undefined) ?? [];
    const newMeshes = object.meshes?.filter((m) => m != undefined) ?? [];
    const newMeshPoses = object.mesh_poses?.filter((p) => p != undefined) ?? [];
    const newPlanes = object.planes?.filter((p) => p != undefined) ?? [];
    const newPlanePoses = object.plane_poses?.filter((p) => p != undefined) ?? [];

    const totalNewShapes = newPrimitives.length + newMeshes.length + newPlanes.length;
    if (totalNewShapes === 0) {
      throw new Error(
        `APPEND operation for object '${objectId}' has no geometry to append. Provide at least one primitive, mesh, or plane.`,
      );
    }

    // Validate pose arrays match geometry arrays
    if (
      newPrimitives.length > 0 &&
      newPrimitivePoses.length > 0 &&
      newPrimitives.length !== newPrimitivePoses.length
    ) {
      throw new Error(
        `APPEND operation for object '${objectId}': primitive count (${newPrimitives.length}) does not match primitive pose count (${newPrimitivePoses.length})`,
      );
    }

    if (
      newMeshes.length > 0 &&
      newMeshPoses.length > 0 &&
      newMeshes.length !== newMeshPoses.length
    ) {
      throw new Error(
        `APPEND operation for object '${objectId}': mesh count (${newMeshes.length}) does not match mesh pose count (${newMeshPoses.length})`,
      );
    }

    if (
      newPlanes.length > 0 &&
      newPlanePoses.length > 0 &&
      newPlanes.length !== newPlanePoses.length
    ) {
      throw new Error(
        `APPEND operation for object '${objectId}': plane count (${newPlanes.length}) does not match plane pose count (${newPlanePoses.length})`,
      );
    }

    try {
      const mergedObject: CollisionObject = {
        ...existingObject,
        // Append new primitives
        primitives: [
          ...existingObject.primitives,
          ...newPrimitives,
        ] as CollisionObject["primitives"],
        primitive_poses: [
          ...existingObject.primitive_poses,
          ...newPrimitivePoses,
        ] as CollisionObject["primitive_poses"],
        // Append new meshes
        meshes: [...existingObject.meshes, ...newMeshes] as CollisionObject["meshes"],
        mesh_poses: [
          ...existingObject.mesh_poses,
          ...newMeshPoses,
        ] as CollisionObject["mesh_poses"],
        // Append new planes
        planes: [...existingObject.planes, ...newPlanes] as CollisionObject["planes"],
        plane_poses: [
          ...existingObject.plane_poses,
          ...newPlanePoses,
        ] as CollisionObject["plane_poses"],
        // Update header and pose if provided
        header: object.header ? this.ensureFullHeader(object.header) : existingObject.header,
        pose: object.pose ? this.ensureFullPose(object.pose) : existingObject.pose,
      };

      // Update color and opacity from planning scene if available (colors may have changed)
      const sceneColor = this.getObjectColorFromScene(objectId);
      if (sceneColor) {
        renderable.userData.settings.color = sceneColor.color;
        // Apply scene opacity multiplicatively
        const messageOpacity = sceneColor.opacity;
        renderable.userData.settings.opacity = this.settings.sceneOpacity * messageOpacity;
      }

      // Update the renderable with the merged data
      renderable.update(mergedObject);

      const addedPrimitives = newPrimitives.length;
      const addedMeshes = newMeshes.length;
      const addedPlanes = newPlanes.length;
      const totalExisting =
        existingObject.primitives.length +
        existingObject.meshes.length +
        existingObject.planes.length;

      log.info(
        `Appended to collision object '${objectId}': ${addedPrimitives} primitives, ${addedMeshes} meshes, ${addedPlanes} planes (total shapes: ${totalExisting + totalNewShapes})`,
      );
    } catch (error) {
      throw new Error(
        `APPEND operation for object '${objectId}' failed: ${error instanceof Error ? error.message : String(error)}`,
      );
    }
  }

  public override removeAllRenderables(): void {
    for (const renderable of this.renderables.values()) {
      this.remove(renderable);
      renderable.dispose();
    }
    this.renderables.clear();

    // Clear all object hashes when removing all renderables for performance optimization
    this.objectHashes.clear();

    // Clear all collision object errors
    const instanceId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
    if (instanceId) {
      this.renderer.settings.errors.clearPath(["layers", instanceId, "collisionObjects"]);
    }
  }

  // Public method to manually retry fetching initial scene
  public retryFetchInitialScene(): void {
    this.initialSceneFetched = false;
    this.fetchingInitialScene = false;
    void this.fetchInitialScene();
  }

  // Public method to check if initial scene has been fetched
  public hasInitialScene(): boolean {
    return this.initialSceneFetched && this.currentScene != undefined;
  }

  // Helper function to ensure a partial pose becomes a full pose
  private ensureFullPose(partialPose: PartialMessage<CollisionObject>["pose"]): Pose {
    if (!partialPose) {
      throw new Error(t("threeDee:poseRequired"));
    }

    return {
      position: {
        x: partialPose.position?.x ?? 0,
        y: partialPose.position?.y ?? 0,
        z: partialPose.position?.z ?? 0,
      },
      orientation: {
        x: partialPose.orientation?.x ?? 0,
        y: partialPose.orientation?.y ?? 0,
        z: partialPose.orientation?.z ?? 0,
        w: partialPose.orientation?.w ?? 1,
      },
    };
  }

  // Helper function to ensure a partial header becomes a full header
  private ensureFullHeader(partialHeader: PartialMessage<CollisionObject>["header"]): Header {
    if (!partialHeader) {
      throw new Error(t("threeDee:headerRequired"));
    }

    return {
      frame_id: partialHeader.frame_id ?? "",
      stamp: {
        sec: partialHeader.stamp?.sec ?? 0,
        nsec: partialHeader.stamp?.nsec ?? 0,
      },
      seq: partialHeader.seq,
    };
  }

  // Helper function to ensure a partial collision object becomes a full collision object
  private ensureFullCollisionObject(
    partialObject: PartialMessage<CollisionObject>,
  ): CollisionObject {
    if (!partialObject.id) {
      throw new Error(t("threeDee:collisionObjectMissingId"));
    }

    if (partialObject.operation == undefined) {
      throw new Error(`Collision object '${partialObject.id}' missing required 'operation' field`);
    }

    return {
      header: this.ensureFullHeader(partialObject.header),
      pose: this.ensureFullPose(partialObject.pose),
      id: partialObject.id,
      type: {
        key: partialObject.type?.key ?? "",
        db: partialObject.type?.db ?? "",
      },
      // Defer shape validation to render-time; coerce to typed-but-invalid sentinels when needed
      primitives:
        partialObject.primitives?.map((primitive) => {
          const typeValue =
            typeof primitive?.type === "number" ? (primitive.type as number) : (-1 as number);
          const dims = Array.isArray(primitive?.dimensions)
            ? normalizeDimensions(primitive.dimensions)
            : [];
          return {
            type: typeValue,
            dimensions: dims,
          } as SolidPrimitive;
        }) ?? [],
      primitive_poses:
        partialObject.primitive_poses?.map((pose) => this.ensureFullPose(pose)) ?? [],
      meshes:
        partialObject.meshes?.map((mesh) => {
          const vertices = Array.isArray(mesh?.vertices)
            ? (mesh.vertices.filter(
                (v): v is { x: number; y: number; z: number } => v != undefined,
              ) as { x: number; y: number; z: number }[])
            : ([] as { x: number; y: number; z: number }[]);
          const triangles = Array.isArray(mesh?.triangles)
            ? (mesh.triangles.map((triangle) => {
                const indices = triangle?.vertex_indices
                  ? normalizeVertexIndices(triangle.vertex_indices)
                  : [];
                return { vertex_indices: new Uint32Array(indices) };
              }) as { vertex_indices: Uint32Array }[])
            : ([] as { vertex_indices: Uint32Array }[]);
          return { vertices, triangles } as Mesh;
        }) ?? [],
      mesh_poses: partialObject.mesh_poses?.map((pose) => this.ensureFullPose(pose)) ?? [],
      planes:
        partialObject.planes?.map((plane) => {
          const coef =
            Array.isArray(plane?.coef) && plane.coef.length === 4
              ? (plane.coef as [number, number, number, number])
              : ([NaN, NaN, NaN, NaN] as [number, number, number, number]);
          return { coef } as { coef: [number, number, number, number] };
        }) ?? [],
      plane_poses: partialObject.plane_poses?.map((pose) => this.ensureFullPose(pose)) ?? [],
      subframe_names:
        partialObject.subframe_names?.filter((name): name is string => name != undefined) ?? [],
      subframe_poses: partialObject.subframe_poses?.map((pose) => this.ensureFullPose(pose)) ?? [],
      operation: partialObject.operation,
    };
  }

  #updatePlanningSceneLayer(
    instanceId: string,
    settings: Partial<LayerSettingsPlanningScene> | undefined,
  ): void {
    // Handle deletes
    if (settings == undefined) {
      // Remove all renderables associated with this layer instance
      // Since planning scene extension shares renderables across all instances,
      // we need to check if this is the last instance before removing renderables
      const remainingInstances = Object.entries(this.renderer.config.layers).filter(
        ([id, config]) => id !== instanceId && config?.layerId === LAYER_ID,
      );

      if (remainingInstances.length === 0) {
        // This was the last planning scene layer, remove all renderables
        for (const renderable of this.renderables.values()) {
          renderable.dispose();
          this.remove(renderable);
        }
        this.renderables.clear();
        this.currentScene = undefined;
        this.initialSceneFetched = false;
        this.fetchingInitialScene = false;
      }

      return;
    }

    // If there are existing renderables, update their settings from the layer config
    this.#reloadRenderableSettings();
  }

  // Helper method to reload settings for existing renderables from layer config
  #reloadRenderableSettings(): void {
    // Also reload the main settings to get updated scene opacity
    const currentInstanceId = this.currentInstanceId ?? this.findFirstPlanningSceneInstanceId();
    if (currentInstanceId) {
      this.settings = this.loadSettingsFromConfig(currentInstanceId);
    }

    for (const [objectId, renderable] of this.renderables.entries()) {
      // Find the instance ID for this renderable
      const instanceId = this.findInstanceIdForCollisionObject(objectId);
      if (!instanceId) {
        continue;
      }

      // Get the layer config
      const layerConfig = this.renderer.config.layers[instanceId] as
        | Partial<LayerSettingsPlanningScene>
        | undefined;
      const savedObjectSettings = layerConfig?.collisionObjects?.[objectId];

      if (savedObjectSettings) {
        // Update the renderable's settings with saved values
        const settings = renderable.userData.settings;
        if (savedObjectSettings.visible != undefined) {
          settings.visible = savedObjectSettings.visible;
        }
        if (savedObjectSettings.color != undefined) {
          settings.color = savedObjectSettings.color;
        }
      }

      // Recalculate opacity based on updated scene opacity
      const sceneColor = this.getObjectColorFromScene(objectId);
      const messageOpacity = sceneColor?.opacity ?? 1.0;
      renderable.userData.settings.opacity = this.settings.sceneOpacity * messageOpacity;

      // Update the visual representation
      renderable.update(renderable.userData.collisionObject);
    }
  }

  // Helper method to find the first planning scene instance ID
  private findFirstPlanningSceneInstanceId(): string | undefined {
    for (const [instanceId, layerConfig] of Object.entries(this.renderer.config.layers)) {
      if (layerConfig?.layerId === LAYER_ID) {
        return instanceId;
      }
    }
    return undefined;
  }

  // Helper method to toggle visibility of all collision objects for a specific layer instance
  // eslint-disable-next-line @lichtblick/no-boolean-parameters
  #toggleCollisionObjectsVisibility(instanceId: string, visible: boolean): void {
    // Update all collision objects for this layer instance
    for (const [objectId, renderable] of this.renderables.entries()) {
      const objectInstanceId = this.findInstanceIdForCollisionObject(objectId);
      if (objectInstanceId !== instanceId) {
        continue;
      }

      // Set object-level visibility only
      renderable.userData.settings.visible = visible;

      // If showing all, also recurse into shapes and set them visible
      if (visible) {
        for (const [, shape] of renderable.userData.shapes.entries()) {
          shape.userData.settings.visible = true;
          shape.visible = true;
        }
      }
    }

    // Update the configuration to persist the changes
    for (const [objectId, renderable] of this.renderables.entries()) {
      const objectInstanceId = this.findInstanceIdForCollisionObject(objectId);
      if (objectInstanceId !== instanceId) {
        continue;
      }

      this.saveCollisionObjectSetting(objectId, "visible", visible);

      // If showing all, also save shape visibility settings
      if (visible) {
        for (const shapeKey of renderable.userData.shapes.keys()) {
          // Save each shape's visibility setting
          this.#handleShapeVisibilityUpdate(objectId, shapeKey, true);
        }
      }
    }
  }

  // Custom layer handlers
  #handleAddPlanningScene = (instanceId: string): void => {
    // Check if a planning scene already exists
    const existingPlanningSceneId = this.findFirstPlanningSceneInstanceId();
    if (existingPlanningSceneId) {
      const errorMessage = t("threeDee:onlyOnePlanningSceneAllowed");

      // Display error message to user if available
      if (this.renderer.displayTemporaryError) {
        try {
          // Use setTimeout to ensure the error is displayed after the current execution context
          setTimeout(() => {
            this.renderer.displayTemporaryError!(errorMessage);
            log.info("Successfully called displayTemporaryError with message:", errorMessage);
          }, 0);
        } catch (error) {
          log.error("Error calling displayTemporaryError:", error);
        }
      } else {
        log.warn("No displayTemporaryError function available to display error message");
      }

      return; // Exit early without creating a new planning scene
    }

    const config: LayerSettingsPlanningScene = {
      ...DEFAULT_CUSTOM_SETTINGS,
      instanceId,
      label: t("threeDee:planningScene"), // Use i18n for the default label
      topic: this.topic || DEFAULT_PLANNING_SCENE_TOPIC, // Use current topic or default
    };

    // Add this instance to the config
    this.renderer.updateConfig((draft) => {
      const maxOrderLayer = _.maxBy(Object.values(draft.layers), (layer) => layer?.order);
      const order = 1 + (maxOrderLayer?.order ?? 0);
      draft.layers[instanceId] = { ...config, order };
    });

    // Update the planning scene layer
    this.#updatePlanningSceneLayer(instanceId, config);

    // Update the settings tree
    this.updateSettingsTree();
    this.renderer.updateCustomLayersCount();
  };

  #handleLayerSettingsAction = (action: SettingsTreeAction): void => {
    const path = action.payload.path;

    // Handle menu actions (duplicate / delete / retry service)
    if (action.action === "perform-node-action" && path.length === 2) {
      const instanceId = path[1]!;
      if (action.payload.id === "delete") {
        // Remove this instance from the config
        this.renderer.updateConfig((draft) => {
          delete draft.layers[instanceId];
        });

        // Remove the layer
        this.#updatePlanningSceneLayer(instanceId, undefined);

        // Update the settings tree
        this.updateSettingsTree();
        this.renderer.updateCustomLayersCount();
      } else if (action.payload.id === "refetch") {
        // Refetch the planning scene
        this.retryFetchInitialScene();
      }
    } else if (
      action.action === "perform-node-action" &&
      path.length === 3 &&
      path[2] === "collisionObjects"
    ) {
      // Handle collision objects show-all/hide-all actions
      const instanceId = path[1]!;
      if (action.payload.id === "show-all") {
        // Show all collision objects
        this.#toggleCollisionObjectsVisibility(instanceId, true);
      } else if (action.payload.id === "hide-all") {
        // Hide all collision objects
        this.#toggleCollisionObjectsVisibility(instanceId, false);
      }
    } else if (action.action === "perform-node-action") {
      // Handle service retry action for custom layers
      const { id } = action.payload;
      if (
        path.length === 3 &&
        path[2] === "service" &&
        (id === "retryService" || id === "refetchService")
      ) {
        this.retryFetchInitialScene();
      }
    } else {
      const value = action.payload.value;

      if (path.length === 3 && path[2] === "visible") {
        // Layer visibility toggle - path is ["layers", instanceId, "visible"]
        this.saveSetting(path, value);
        // Visibility will be recalculated in the next frame by startFrame()
        // No need to manually update extension visibility here
        this.updateSettingsTree();
      } else if (path.length === 3) {
        // Layer field settings (defaultColor, sceneOpacity, etc.)
        this.#handleLayerSettingsUpdate(action);
      } else if (path.length === 4 && path[2] === "collisionObjects") {
        // Collision object visibility toggle (eye icon) - path is ["layers", instanceId, "collisionObjects", objectId]
        const objectId = path[3];
        if (typeof objectId === "string") {
          this.handleCollisionObjectSettingsUpdate(objectId, "visible", value);
        }
      } else if (path.length === 5 && path[2] === "collisionObjects") {
        // Collision object field settings - path is ["layers", instanceId, "collisionObjects", objectId, fieldName]
        const objectId = path[3];
        const fieldName = path[4]; // The actual field name is in the path, not the input

        if (typeof objectId === "string" && typeof fieldName === "string") {
          // For eye icon toggles, input is "boolean" but fieldName is "visible"
          this.handleCollisionObjectSettingsUpdate(objectId, fieldName, value);
        }
      } else if (path.length === 6 && path[2] === "collisionObjects") {
        // Per-shape visibility: ["layers", instanceId, "collisionObjects", objectId, shapeKey, "visible"]
        const objectId = path[3];
        const shapeKey = path[4];
        const leaf = path[5];
        if (leaf === "visible" && typeof objectId === "string" && typeof shapeKey === "string") {
          this.#handleShapeVisibilityUpdate(objectId, shapeKey, Boolean(value));
        }
      }
    }
  };

  // Update per-shape visibility and persist it to config
  // eslint-disable-next-line @lichtblick/no-boolean-parameters
  #handleShapeVisibilityUpdate(objectId: string, shapeKey: string, visible: boolean): void {
    const renderable = this.renderables.get(objectId);
    if (!renderable) {
      return;
    }

    const shape = renderable.userData.shapes.get(shapeKey);
    if (shape) {
      shape.userData.settings.visible = visible;
      shape.visible = visible;
    }

    // Persist to config under the layer instance
    const instanceId = this.findInstanceIdForCollisionObject(objectId);
    if (!instanceId) {
      return;
    }

    this.renderer.updateConfig((draft) => {
      const layerConfig = draft.layers[instanceId] as LayerSettingsPlanningScene | undefined;
      if (!layerConfig) {
        return;
      }
      layerConfig.collisionObjects = layerConfig.collisionObjects ?? {};
      const objConfig = (layerConfig.collisionObjects[objectId] =
        layerConfig.collisionObjects[objectId] ?? {});
      objConfig.shapes = objConfig.shapes ?? {};
      (objConfig.shapes[shapeKey] = objConfig.shapes[shapeKey] ?? { visible: true }).visible =
        visible;
    });

    // Update settings tree to reflect new state
    this.updateSettingsTree();
  }

  #handleLayerSettingsUpdate = (action: { action: "update" } & SettingsTreeAction): void => {
    const path = action.payload.path;

    if (path.length !== 3) {
      return; // Doesn't match the pattern of ["layers", instanceId, field]
    }

    const { value } = action.payload;
    const fieldName = path[2]; // The field name is the third element in the path

    // Save the setting to config first
    this.saveSetting(path, value);

    // Handle specific settings that need special processing
    if (fieldName === "defaultColor") {
      // Update all collision objects that don't have colors in the planning scene message
      for (const renderable of this.renderables.values()) {
        const objectId = renderable.userData.collisionObject.id;
        const sceneColor = this.getObjectColorFromScene(objectId);

        // Only update if no color is specified in the planning scene message
        if (!sceneColor) {
          renderable.userData.settings.color = value as string;
          // Trigger visual update
          renderable.update(renderable.userData.collisionObject);
        }
      }
    } else if (fieldName === "sceneOpacity") {
      // Update all collision objects with new scene opacity
      const newSceneOpacity = value as number;

      for (const renderable of this.renderables.values()) {
        const objectId = renderable.userData.collisionObject.id;
        const sceneColor = this.getObjectColorFromScene(objectId);
        const messageOpacity = sceneColor?.opacity ?? 1.0;

        // Apply scene opacity multiplicatively
        renderable.userData.settings.opacity = newSceneOpacity * messageOpacity;
        log.info(
          `Message opacity: ${messageOpacity}, New scene opacity: ${newSceneOpacity}, Renderable opacity: ${renderable.userData.settings.opacity}`,
        );

        // Trigger visual update to apply new opacity
        renderable.update(renderable.userData.collisionObject);
      }
    } else if (fieldName === "showCollisionObjects") {
      // Visibility will be recalculated in the next frame by startFrame()
      // No need to manually update all objects here
    } else if (fieldName === "showAttachedObjects") {
      // Visibility will be recalculated in the next frame by startFrame()
      // No need to manually update all objects here
    } else if (fieldName === "showOctomap") {
      // Check if user is trying to enable octomap
      if (value === true) {
        // Show temporary error and revert the setting
        this.renderer.displayTemporaryError!(t("threeDee:octomapNotImplemented"));

        // Revert the setting back to false
        this.saveSetting(path, false);

        // Update the settings tree to reflect the reverted value
        this.updateSettingsTree();
      }
    } else if (fieldName === "topic") {
      // Topic changed - clear current scene and refetch from new topic
      const newTopic = value as string;
      log.info(`Planning scene topic changed to: ${newTopic}`);

      // Clear current scene data
      this.currentScene = undefined;
      this.initialSceneFetched = false;
      this.fetchingInitialScene = false;

      // Clear all renderables since we're switching topics
      this.removeAllRenderables();

      // Update the internal topic tracking
      this.topic = newTopic;

      // The subscription will be updated automatically by the renderer
      // when the settings tree is updated
    }
  };
}
