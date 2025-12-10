"use client";

import React, { Suspense } from "react";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { OrbitControls, Bounds, Center, Html } from "@react-three/drei";
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

type AutoRotateAxis = "x" | "y" | "z";

const MATERIAL_COLOR_MAP: Record<string, string> = {
  "steel_-_satin": "#9ba3b1",
  "stainless_steel_-_satin": "#cfd6e3",
  "opaque(75,75,75)": "#2a2d33",
  "opaque(10,10,10)": "#0e0f14",
  "opaque(25,25,25)": "#16181f",
  "opaque(221,232,255)": "#baceff",
  "opaque(229,234,237)": "#d1d9e2",
  "opaque(203,210,239)": "#9fb4ff",
  "opaque(163,170,173)": "#7b848f",
  "plastic_-_matte_(red)": "#c44532",
  "plastic_-_matte_(black)": "#1b1d23",
  "plastic_-_matte_(white)": "#f1f3f6",
  "0.231373_0.380392_0.705882_0.000000_0.000000": "#4a70d0",
  "0.615686_0.811765_0.929412_0.000000_0.000000": "#8fc1ff",
  "0.647059_0.647059_0.647059_0.000000_0.000000": "#8f979f",
  "0.972549_0.529412_0.003922_0.000000_0.000000": "#ff8d1f",
};

function applyMaterialOverrides(material: THREE.MeshStandardMaterial) {
  const nameKey = material.name?.toLowerCase();
  if (nameKey && MATERIAL_COLOR_MAP[nameKey]) {
    material.color.set(MATERIAL_COLOR_MAP[nameKey]);
  }

  if (!material.color || material.color.getHexString() === "cccccc") {
    material.color.set("#7f8a94");
  }

  material.metalness = Math.min(material.metalness ?? 0.2, 0.45);
  material.roughness = Math.min(Math.max(material.roughness ?? 0.55, 0.35), 0.75);
  material.needsUpdate = true;
}

function toStandardMaterial(material: THREE.Material): THREE.Material {
  if (material instanceof THREE.MeshStandardMaterial) {
    const clone = material.clone();
    clone.needsUpdate = true;
    applyMaterialOverrides(clone);
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

  applyMaterialOverrides(standard);
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
      applyMaterialOverrides(child.material as THREE.MeshStandardMaterial);
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

function AutoRotator({
  axis = "y",
  speed = 0.3,
  children,
}: {
  axis?: AutoRotateAxis;
  speed?: number;
  children: React.ReactNode;
}) {
  const groupRef = React.useRef<THREE.Group>(null);

  useFrame((_, delta) => {
    if (!groupRef.current) {
      return;
    }

    if (!speed) {
      return;
    }

    const target = groupRef.current.rotation;
    const deltaRotation = delta * speed;

    if (axis === "x") {
      target.x += deltaRotation;
    } else if (axis === "y") {
      target.y += deltaRotation;
    } else {
      target.z += deltaRotation;
    }
  });

  return <group ref={groupRef}>{children}</group>;
}

function LoadingIndicator({ accent }: { accent?: string }) {
  const baseColor = React.useMemo(() => new THREE.Color(accent ?? "#7fffd4"), [accent]);
  const ringColor = `#${baseColor.clone().lerp(new THREE.Color("#ffffff"), 0.5).getHexString()}33`;
  const glowColor = `#${baseColor.clone().lerp(new THREE.Color("#0a0c1c"), 0.2).getHexString()}55`;
  const primaryHex = `#${baseColor.getHexString()}`;

  return (
    <Html center>
      <div style={{ position: "relative", width: 48, height: 48 }}>
        <style>{`
          @keyframes objViewerSpin { to { transform: rotate(360deg); } }
          @keyframes objViewerPulse { 0%, 100% { transform: scale(0.6); opacity: 0.35; } 50% { transform: scale(1); opacity: 0.8; } }
        `}</style>
        <div
          style={{
            position: "absolute",
            inset: 0,
            borderRadius: "50%",
            background: `radial-gradient(circle, ${glowColor} 0%, transparent 65%)`,
            filter: "blur(6px)",
          }}
        />
        <div
          style={{
            position: "absolute",
            inset: 0,
            borderRadius: "50%",
            border: `3px solid ${ringColor}`,
            borderTopColor: primaryHex,
            animation: "objViewerSpin 1.2s linear infinite",
          }}
        />
        <div
          style={{
            position: "absolute",
            top: "50%",
            left: "50%",
            width: 10,
            height: 10,
            borderRadius: "50%",
            backgroundColor: primaryHex,
            transform: "translate(-50%, -50%)",
            animation: "objViewerPulse 1.6s ease-in-out infinite",
          }}
        />
      </div>
    </Html>
  );
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
  autoRotateAxis?: AutoRotateAxis;
  autoRotateSpeed?: number;
};

export default function ObjViewer({
  src,
  mtlSrc,
  height = 320,
  background = "#11131f",
  fallbackColor,
  autoRotateAxis,
  autoRotateSpeed,
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
        <Suspense fallback={<LoadingIndicator accent={fallbackColor} />}>
          <Bounds fit clip observe margin={1.2}>
            {autoRotateAxis ? (
              <AutoRotator axis={autoRotateAxis} speed={autoRotateSpeed}>
                <Model src={src} mtlSrc={mtlSrc} fallbackColor={fallbackColor} />
              </AutoRotator>
            ) : (
              <Model src={src} mtlSrc={mtlSrc} fallbackColor={fallbackColor} />
            )}
          </Bounds>
        </Suspense>

        {/* User camera controls */}
        <OrbitControls enableDamping dampingFactor={0.1} />
      </Canvas>
    </div>
  );
}
