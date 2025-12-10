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

function toStandardMaterial(material: THREE.Material): THREE.Material {
  if (material instanceof THREE.MeshStandardMaterial) {
    const clone = material.clone();
    clone.needsUpdate = true;
    return clone;
  }

  const hasMetalness = material as THREE.Material & { metalness?: number };
  const hasRoughness = material as THREE.Material & { roughness?: number };
  const hasColor = material as THREE.Material & { color?: THREE.Color };
  const hasEmissive = material as THREE.Material & { emissive?: THREE.Color };
  const hasMap = material as THREE.Material & {
    map?: THREE.Texture | null;
    normalMap?: THREE.Texture | null;
    alphaMap?: THREE.Texture | null;
  };

  const standard = new THREE.MeshStandardMaterial({
    color:
      hasColor.color instanceof THREE.Color
        ? hasColor.color.clone()
        : new THREE.Color("#cccccc"),
    metalness:
      typeof hasMetalness.metalness === "number"
        ? hasMetalness.metalness ?? 0.2
        : 0.2,
    roughness:
      typeof hasRoughness.roughness === "number"
        ? hasRoughness.roughness ?? 0.55
        : 0.55,
    transparent: material.transparent ?? false,
    opacity: material.opacity ?? 1,
    side: material.side ?? THREE.FrontSide,
  });

  if (hasEmissive.emissive instanceof THREE.Color) {
    standard.emissive.copy(hasEmissive.emissive);
  }

  if (hasMap.map) {
    const texture = hasMap.map;
    standard.map = texture;
    texture.needsUpdate = true;
  }

  if (hasMap.normalMap) {
    const texture = hasMap.normalMap;
    standard.normalMap = texture;
    texture.needsUpdate = true;
  }

  if (hasMap.alphaMap) {
    const texture = hasMap.alphaMap;
    standard.alphaMap = texture;
    standard.transparent = true;
    texture.needsUpdate = true;
  }

  if (material.name) {
    standard.name = material.name;
  }

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
        child.material = child.material.map((mat) => toStandardMaterial(mat));
      } else if (child.material) {
        child.material = toStandardMaterial(child.material);
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
        gl={{ outputColorSpace: THREE.SRGBColorSpace }}
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
