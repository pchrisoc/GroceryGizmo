"use client";

import React, { Suspense } from "react";
import { Canvas, useLoader } from "@react-three/fiber";
import { OrbitControls, Bounds, Center } from "@react-three/drei";
import * as THREE from "three";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { MTLLoader } from "three/examples/jsm/loaders/MTLLoader.js";

// --------------------------------------------------------
// Load OBJ model
// --------------------------------------------------------
type ModelProps = {
  src: string;
  mtlSrc?: string;
  fallbackColor?: string;
};

function convertToStandardMaterial(
  material: THREE.Material
): THREE.MeshStandardMaterial {
  if (material instanceof THREE.MeshStandardMaterial) {
    return material;
  }

  const color =
    "color" in material
      ? ((material as THREE.Material & { color?: THREE.Color }).color ?? null)
      : null;
  const emissive =
    "emissive" in material
      ? ((material as THREE.Material & { emissive?: THREE.Color }).emissive ?? null)
      : null;
  const emissiveIntensity =
    "emissiveIntensity" in material
      ? ((material as THREE.Material & { emissiveIntensity?: number }).emissiveIntensity ?? undefined)
      : undefined;
  const map =
    "map" in material
      ? ((material as THREE.Material & { map?: THREE.Texture | null }).map ?? null)
      : null;
  const normalMap =
    "normalMap" in material
      ? ((material as THREE.Material & { normalMap?: THREE.Texture | null }).normalMap ?? null)
      : null;
  const roughness =
    "roughness" in material
      ? (material as THREE.Material & { roughness?: number }).roughness
      : undefined;
  const metalness =
    "metalness" in material
      ? (material as THREE.Material & { metalness?: number }).metalness
      : undefined;
  const flatShading =
    "flatShading" in material
      ? Boolean((material as THREE.Material & { flatShading?: boolean }).flatShading)
      : false;
  const vertexColors =
    "vertexColors" in material
      ? Boolean((material as THREE.Material & { vertexColors?: boolean }).vertexColors)
      : false;

  const standard = new THREE.MeshStandardMaterial({
    color: color ? color.clone() : new THREE.Color("#ffffff"),
    map: map ?? undefined,
    metalness: metalness ?? 0.15,
    roughness: roughness ?? 0.55,
  });

  if (normalMap) {
    standard.normalMap = normalMap;
  }

  standard.side = material.side;
  standard.transparent = material.transparent ?? false;
  standard.opacity = material.opacity ?? 1;
  if ("alphaTest" in material) {
    standard.alphaTest = (material as THREE.Material & { alphaTest?: number }).alphaTest ?? 0;
  }

  standard.flatShading = flatShading;
  standard.vertexColors = vertexColors;

  if (emissive) {
    standard.emissive.copy(emissive);
    if (typeof emissiveIntensity === "number") {
      standard.emissiveIntensity = emissiveIntensity;
    }
  }

  if (standard.map) {
    standard.map.needsUpdate = true;
  }
  if (standard.normalMap) {
    standard.normalMap.needsUpdate = true;
  }

  standard.name = material.name;
  standard.needsUpdate = true;

  return standard;
}

function ModelWithoutMaterials({
  src,
  fallbackColor,
}: {
  src: string;
  fallbackColor?: string;
}) {
  const obj = useLoader(OBJLoader, src) as THREE.Group;

  obj.traverse((child: THREE.Object3D) => {
    if (child instanceof THREE.Mesh) {
      child.material = new THREE.MeshStandardMaterial({
        color: fallbackColor ?? "#cccccc",
        metalness: 0.2,
        roughness: 0.35,
      });
      child.castShadow = false;
      child.receiveShadow = false;
    }
  });

  return (
    <Center>
      <primitive object={obj} scale={1} />
    </Center>
  );
}

function ModelWithMaterials({ src, mtlSrc }: { src: string; mtlSrc: string }) {
  const materials = useLoader(MTLLoader, mtlSrc, (loader) => {
    loader.crossOrigin = "anonymous";
  });

  const obj = useLoader(
    OBJLoader,
    src,
    (loader) => {
      materials.preload();
      loader.setMaterials(materials);
    }
  ) as THREE.Group;

  obj.traverse((child: THREE.Object3D) => {
    if (child instanceof THREE.Mesh) {
      if (Array.isArray(child.material)) {
        child.material = child.material.map((mat) =>
          convertToStandardMaterial(mat)
        );
      } else if (child.material) {
        child.material = convertToStandardMaterial(child.material);
      }
      child.castShadow = false;
      child.receiveShadow = false;
    }
  });

  return (
    <Center>
      <primitive object={obj} scale={1} />
    </Center>
  );
}

function Model({ src, mtlSrc, fallbackColor }: ModelProps) {
  if (mtlSrc) {
    return <ModelWithMaterials src={src} mtlSrc={mtlSrc} />;
  }

  return <ModelWithoutMaterials src={src} fallbackColor={fallbackColor} />;
}

// --------------------------------------------------------
// OBJ Viewer Component
// --------------------------------------------------------
export type ObjViewerProps = {
  src: string;
  mtlSrc?: string;
  height?: number;
  background?: string;
  fallbackColor?: string;
};

export default function ObjViewer({
  src,
  mtlSrc,
  height = 320,
  background = "#11131f",
  fallbackColor,
}: ObjViewerProps) {
  return (
    <div style={{ width: "100%", height }}>
      <Canvas
        key={`${src}-${mtlSrc ?? "default"}`}
        camera={{ position: [2.5, 2, 2], fov: 45 }}
        shadows={false}
        style={{ background }}
      >
        {/* Lighting */}
        <ambientLight intensity={0.4} />
        <hemisphereLight
          args={["#f0f4ff", "#1a1b25", 0.9]}
        />
        <directionalLight
          position={[6, 6, 6]}
          intensity={0.9}
          castShadow
          shadow-mapSize-width={2048}
          shadow-mapSize-height={2048}
        />
        <directionalLight position={[-6, 4, -6]} intensity={0.6} />

        {/* Load the OBJ */}
        <Suspense fallback={null}>
          <Bounds fit clip observe margin={1.2}>
            <Model src={src} mtlSrc={mtlSrc} fallbackColor={fallbackColor} />
          </Bounds>
        </Suspense>

        {/* User camera controls */}
        <OrbitControls enableDamping dampingFactor={0.1} />
      </Canvas>
    </div>
  );
}
