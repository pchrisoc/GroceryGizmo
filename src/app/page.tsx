"use client";

import type { SxProps, Theme } from "@mui/material/styles";
import React from "react";
import {
  AppBar,
  Avatar,
  Box,
  Button,
  Card,
  CardActionArea,
  CardContent,
  Container,
  CssBaseline,
  Drawer,
  Dialog,
  DialogContent,
  Divider,
  IconButton,
  ImageList,
  ImageListItem,
  List,
  ListItem,
  ListItemButton,
  ListItemText,
  Link as MuiLink,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  Toolbar,
  Typography,
  Grid,
  FormControl,
  InputLabel,
  MenuItem,
} from "@mui/material";
import MenuIcon from "@mui/icons-material/Menu";
import CloseIcon from "@mui/icons-material/Close";
import AddIcon from "@mui/icons-material/Add";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import Image from "next/image";
import dynamic from "next/dynamic";
import type { ObjViewerProps } from "./components/ObjViewer";
import Select, { SelectChangeEvent } from "@mui/material/Select";

const ObjViewer = dynamic<ObjViewerProps>(
  () => import("./components/ObjViewer"),
  { ssr: false }
);

function stringToAvatarColor(name: string): string {
  let hash = 0;
  for (let i = 0; i < name.length; i += 1) {
    hash = name.charCodeAt(i) + ((hash << 5) - hash);
  }

  let color = "#";
  for (let i = 0; i < 3; i += 1) {
    const value = (hash >> (i * 8)) & 0xff;
    const normalized = (value % 128) + 96;
    color += `00${normalized.toString(16)}`.slice(-2);
  }

  return color;
}

function initialsFromName(name: string): string {
  const parts = name.trim().split(/\s+/);
  if (parts.length === 0) {
    return "";
  }

  const first = parts[0]?.[0] ?? "";
  const second =
    parts.length > 1
      ? parts[parts.length - 1]?.[0] ?? ""
      : parts[0]?.[1] ?? "";

  return `${first}${second}`.toUpperCase();
}

const ACCENT_COLORS = [
  "#7fffd4",
  "#7ab8ff",
  "#ff9bf6",
  "#ffe58a",
  "#ff8f76",
  "#9d8eff",
  "#7dffb5",
  "#ffc49b",
  "#9fe5ff",
  "#ffa2c2",
  "#a4ffdd",
  "#ffd4a8",
];

const REACTIVE_VARIANTS = [
  "lift",
  "tilt",
  "slide",
  "pulse",
  "ripple",
  "swing",
  "prism",
] as const;

const NAV_SECTIONS = [
  { id: "introduction", label: "Introduction" },
  { id: "team", label: "Team" },
  { id: "design", label: "Design" },
  { id: "implementation", label: "Implementation" },
  { id: "code", label: "Code" },
  { id: "viewer", label: "3D Models" },
  { id: "results", label: "Results" },
  { id: "conclusion", label: "Conclusion" },
] as const;

const mobileTeamOverlaySxCache = new Map<string, SxProps<Theme>>();

function getMobileOverlayStyles(accent: string): SxProps<Theme> {
  if (mobileTeamOverlaySxCache.has(accent)) {
    return mobileTeamOverlaySxCache.get(accent)!;
  }

  const styles: SxProps<Theme> = {
    position: "absolute",
    inset: 0,
    display: "flex",
    flexDirection: "column",
    justifyContent: "center",
    gap: 2,
    p: 3,
    background: `linear-gradient(180deg, ${hexToRgba(accent, 0.85)} 0%, rgba(5,7,17,0.96) 100%)`,
    border: `1px solid ${hexToRgba(accent, 0.45)}`,
    boxShadow: `0 22px 48px ${hexToRgba(accent, 0.25)}`,
    color: "#e9fef7",
    transition: "opacity 0.35s ease, transform 0.4s ease",
    transform: "translateY(16px)",
    opacity: 0,
    pointerEvents: "none",
    zIndex: 4,
  };

  mobileTeamOverlaySxCache.set(accent, styles);
  return styles;
}

type ReactiveVariant = (typeof REACTIVE_VARIANTS)[number];

function hashStringToInt(input: string): number {
  let hash = 0;
  for (let i = 0; i < input.length; i += 1) {
    hash = (hash << 5) - hash + input.charCodeAt(i);
    hash |= 0;
  }
  return Math.abs(hash);
}

function accentFromKey(key: string): string {
  const hash = hashStringToInt(key);
  return ACCENT_COLORS[hash % ACCENT_COLORS.length];
}

function variantFromKey(key: string): ReactiveVariant {
  const hash = hashStringToInt(`${key}-variant`);
  return REACTIVE_VARIANTS[hash % REACTIVE_VARIANTS.length];
}

function hexToRgba(hex: string, alpha: number): string {
  const normalized = hex.replace("#", "");
  const bigint = parseInt(normalized, 16);
  const r = (bigint >> 16) & 255;
  const g = (bigint >> 8) & 255;
  const b = bigint & 255;
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

const CODE_THEME_MAP: Record<
  string,
  {
    background: string;
    border: string;
    glow: string;
    text: string;
    badge: string;
    badgeText: string;
  }
> = {
  python: {
    background: "linear-gradient(160deg, rgba(12,38,33,0.92), rgba(6,18,20,0.96))",
    border: "rgba(127,255,212,0.45)",
    glow: "0 22px 48px rgba(127,255,212,0.18)",
    text: "#e9fff7",
    badge: "rgba(127,255,212,0.14)",
    badgeText: "#7fffd4",
  },
  typescript: {
    background: "linear-gradient(160deg, rgba(16,32,58,0.92), rgba(9,15,32,0.96))",
    border: "rgba(122,184,255,0.55)",
    glow: "0 22px 52px rgba(122,184,255,0.2)",
    text: "#f2f6ff",
    badge: "rgba(122,184,255,0.18)",
    badgeText: "#7ab8ff",
  },
  bash: {
    background: "linear-gradient(160deg, rgba(18,39,24,0.92), rgba(8,18,14,0.96))",
    border: "rgba(125,255,181,0.45)",
    glow: "0 22px 48px rgba(125,255,181,0.18)",
    text: "#e8ffef",
    badge: "rgba(125,255,181,0.16)",
    badgeText: "#7dffb5",
  },
  default: {
    background: "linear-gradient(160deg, rgba(24,27,44,0.92), rgba(8,10,19,0.96))",
    border: "rgba(160,174,192,0.4)",
    glow: "0 18px 42px rgba(17,20,38,0.35)",
    text: "#f5f6fb",
    badge: "rgba(160,174,192,0.18)",
    badgeText: "#d5daef",
  },
};

type ScrollReactiveCardProps = React.ComponentProps<typeof Card> & {
  accentKey: string;
  accentColor?: string;
  effectVariant?: ReactiveVariant;
};

function getVariantStyles(
  variant: ReactiveVariant,
  accent: string,
  active: boolean
) {
  const glowStrong = hexToRgba(accent, active ? 0.32 : 0.18);
  switch (variant) {
    case "tilt":
      return {
        transform: active
          ? "translateY(-14px) rotateX(6deg) rotateY(-3deg)"
          : "translateY(20px) rotateX(10deg) rotateY(-6deg)",
        "&::after": {
          content: "\"\"",
          position: "absolute",
          inset: active ? "10%" : "18%",
          borderRadius: 28,
          opacity: active ? 0.6 : 0,
          border: `1px solid ${hexToRgba(accent, active ? 0.5 : 0.22)}`,
          boxShadow: active ? `0 0 55px ${glowStrong}` : "none",
          transition: "all 0.7s var(--gg-curve)",
        },
      } as const;
    case "slide":
      return {
        transform: active
          ? "translateY(-14px) translateX(8px)"
          : "translateY(20px) translateX(-14px)",
        "&::after": {
          content: "\"\"",
          position: "absolute",
          inset: 0,
          background: `linear-gradient(120deg, transparent 0%, ${hexToRgba(
            accent,
            active ? 0.18 : 0
          )} 45%, transparent 90%)`,
          mixBlendMode: "screen",
          opacity: active ? 1 : 0,
          transform: active ? "translateX(0)" : "translateX(-25%)",
          transition: "all 0.7s var(--gg-curve)",
        },
      } as const;
    case "pulse":
      return {
        transform: active ? "translateY(-16px) scale(1.02)" : "translateY(20px)",
        boxShadow: active
          ? `0 28px 65px ${hexToRgba(accent, 0.28)}`
          : `0 0 0 ${hexToRgba(accent, 0.0)}`,
        "&::after": {
          content: "\"\"",
          position: "absolute",
          inset: "-30%",
          borderRadius: "50%",
          border: `2px solid ${hexToRgba(accent, active ? 0.45 : 0.15)}`,
          opacity: active ? 0.55 : 0,
          transition: "opacity 0.7s var(--gg-curve)",
        },
      } as const;
    case "ripple":
      return {
        transform: active ? "translateY(-10px) scale(1.015)" : "translateY(18px)",
        "&::after": {
          content: "\"\"",
          position: "absolute",
          inset: "-15%",
          borderRadius: "50%",
          background: `radial-gradient(circle, ${hexToRgba(
            accent,
            active ? 0.28 : 0.12
          )} 0%, transparent 65%)`,
          opacity: active ? 0.75 : 0,
          filter: "blur(12px)",
          transition: "opacity 0.7s var(--gg-curve)",
        },
      } as const;
    case "swing":
      return {
        transform: active
          ? "translateY(-12px) rotateZ(-2deg)"
          : "translateY(20px) rotateZ(3deg)",
        "&::after": {
          content: "\"\"",
          position: "absolute",
          inset: "8%",
          borderRadius: 24,
          border: `1px dashed ${hexToRgba(accent, active ? 0.5 : 0.2)}`,
          opacity: active ? 0.7 : 0,
          transform: active ? "rotateZ(0deg)" : "rotateZ(6deg)",
          transition: "all 0.7s var(--gg-curve)",
        },
      } as const;
    case "prism":
      return {
        transform: active ? "translateY(-14px)" : "translateY(18px)",
        "&::after": {
          content: "\"\"",
          position: "absolute",
          inset: 0,
          background: `linear-gradient(135deg, ${hexToRgba(
            accent,
            active ? 0.35 : 0
          )} 0%, transparent 45%, ${hexToRgba(accent, active ? 0.22 : 0)} 70%, transparent 100%)`,
          opacity: active ? 1 : 0,
          mixBlendMode: "screen",
          transition: "opacity 0.7s var(--gg-curve)",
        },
      } as const;
    default:
      return {
        transform: active ? "translateY(-12px)" : "translateY(20px)",
        "&::after": {
          content: "\"\"",
          position: "absolute",
          inset: active ? "6%" : "12%",
          borderRadius: 24,
          border: `1px solid ${hexToRgba(accent, active ? 0.35 : 0.15)}`,
          opacity: active ? 0.5 : 0,
          transition: "all 0.6s var(--gg-curve)",
        },
      } as const;
  }
}

function ScrollReactiveCard({
  accentKey,
  accentColor,
  effectVariant,
  children,
  sx,
  ...cardProps
}: ScrollReactiveCardProps) {
  const ref = React.useRef<HTMLDivElement | null>(null);
  const [active, setActive] = React.useState(false);

  React.useEffect(() => {
    const node = ref.current;
    if (!node) {
      return;
    }

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          setActive(entry.isIntersecting);
        });
      },
      { threshold: 0.35 }
    );

    observer.observe(node);

    return () => {
      observer.disconnect();
    };
  }, []);

  const accent = React.useMemo(
    () => accentColor ?? accentFromKey(accentKey),
    [accentColor, accentKey]
  );

  const resolvedVariant = React.useMemo(
    () => effectVariant ?? variantFromKey(accentKey),
    [effectVariant, accentKey]
  );

  const baseStyles = React.useMemo(() => {
    const glowSoft = hexToRgba(accent, active ? 0.38 : 0.08);
    return {
      position: "relative",
      overflow: "hidden",
      borderRadius: 3,
      border: `1px solid ${active ? hexToRgba(accent, 0.45) : "rgba(255,255,255,0.08)"}`,
      transition: "transform 0.75s var(--gg-curve), border-color 0.6s var(--gg-curve), box-shadow 0.75s var(--gg-curve), background 0.75s var(--gg-curve)",
      boxShadow: active ? `0 32px 55px ${hexToRgba(accent, 0.26)}` : "0 0 0 rgba(0,0,0,0)",
      background: active
        ? `linear-gradient(160deg, ${hexToRgba(accent, 0.18)} 0%, rgba(9,11,25,0.96) 70%)`
        : undefined,
      "--gg-curve": "cubic-bezier(0.22, 1, 0.36, 1)",
      "&::before": {
        content: "\"\"",
        position: "absolute",
        inset: "-40% -18%",
        background: `radial-gradient(circle at top, ${glowSoft}, transparent 55%)`,
        opacity: active ? 1 : 0,
        transition: "opacity 0.75s var(--gg-curve)",
        pointerEvents: "none",
      },
      "&::after": {
        pointerEvents: "none",
      },
    } as const;
  }, [accent, active]);

  const variantStyles = React.useMemo(
    () => getVariantStyles(resolvedVariant, accent, active),
    [resolvedVariant, accent, active]
  );

  const combinedSx = React.useMemo(() => {
    const extra = Array.isArray(sx) ? sx : sx ? [sx] : [];
    return [baseStyles, variantStyles, ...extra];
  }, [baseStyles, variantStyles, sx]);

  return (
    <Card ref={ref} sx={combinedSx} {...cardProps}>
      <Box sx={{ position: "relative", zIndex: 1 }}>{children}</Box>
    </Card>
  );
}


// ==================== THEME ====================
const theme = createTheme({
  palette: {
    mode: "dark",
    primary: { main: "#7FFFD4" },
    background: { default: "#050711", paper: "#11131f" },
  },
  typography: {
    fontFamily: ["Inter", "system-ui", "sans-serif"].join(","),
  },
});


// ==================== DATA =====================
const teamMembers: Array<{
  name: string;
  role: string;
  background: string;
  avatar?: string;
}> = [
  {
    name: "Arya Sasikumar",
    role: "Sensing & Execution",
    background:
      "Mechanical Engineering and Business student focused on robotics, with experience in Solidworks, ROS, Python, and C++.",
    avatar: "/arya.png",
  },
  {
    name: "Gursimar Virk",
    role: "Manipulation & CAD Engineer",
    background:
      "Mechanical Engineering major and Materials Science minor who works with humanoid robots and designs with Solidworks, ROS, Python, and C++.",
    avatar: "/gursimar.jpeg",
  },
  {
    name: "Yamuna Rao",
    role: "Perception & Planning",
    background:
      "EECS major interested in robotics and optimization, with experience in computer vision, Python, C, PyTorch, and TensorFlow.",
    avatar: "/yamuna.jpeg",
  },
  {
    name: "Divya Krishnaswamy",
    role: "Testing & Integration",
    background:
      "Bioengineering major and EECS minor interested in surgical robotics, with experience in Solidworks, C++, Python, and embedded systems.",
    avatar: "/divya.jpg",
  },
  {
    name: "Patrick O’Connor",
    role: "Web & CAD Engineer",
    background:
      "Third-year EECS major interested in electronics and robotics, with experience in Fusion, KiCad, and Python.",
    avatar: "/patrick.jpg",
  },
];

const codeSamples = [
  {
    value: "ros-launch",
    label: "ros_launch.py",
    language: "python",
    description:
      "Launches the RealSense camera and ArUco detector nodes that power GroceryGizmo’s perception stack.",
    code: String.raw`import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            parameters=[{"rgb_module.profile": "640x480x30"}],
        ),
        Node(
            package="aruco_detector",
            executable="detector",
            output="screen",
            parameters=[{"publish_tf": True}],
        ),
    ])


if __name__ == "__main__":
    launch.LaunchService(argv=[]).run(generate_launch_description())`,
  },
  {
    value: "api-handler",
    label: "route-handler.ts",
    language: "typescript",
    description:
      "Next.js API route that revalidates cached dashboards whenever a new manipulation log is uploaded.",
    code: String.raw`import type { NextRequest } from "next/server";
import { revalidateTag } from "next/cache";


export async function POST(request: NextRequest) {
    const payload = await request.json();
    if (!payload?.tag) {
        return new Response("Missing tag", { status: 400 });
    }

    await revalidateTag(payload.tag);
    return new Response(JSON.stringify({ ok: true }), {
        headers: { "Content-Type": "application/json" },
    });
}

export const dynamic = "force-dynamic";`,
  },
  {
    value: "deploy-script",
    label: "deploy.sh",
    language: "bash",
    description:
      "Deployment script that installs dependencies, builds the site, and syncs static assets to the robot operator console.",
    code: String.raw`#!/usr/bin/env bash
set -euo pipefail

pnpm install --frozen-lockfile
pnpm run build

rsync -avz --delete \
    ./.next \
    public \
    package.json \
    user@robot:/opt/grocerygizmo/site/

echo "Deployment finished ✅"`,
  },
];

const galleryItems = [
  { src: "/arm.jpg", title: "Omron arm prepared for a pick" },
  { src: "/arm2.jpg", title: "Arm reaching into the fridge" },
  { src: "/arm3.JPG", title: "Close-up of the gripper" },
  { src: "/ARCo_detection.JPEG", title: "AR tag pose detection" },
  { src: "/object_detection.JPEG", title: "Object detection pipeline" },
  { src: "/ros.JPG", title: "ROS graph overview" },
];

const modelGallery = [
  {
    title: "Camera Mount",
    description:
      "Custom Intel RealSense mount designed for the Omron robotic arm.",
    src: "/models/mount.obj",
    mtlSrc: "/models/mount.mtl",
  },

  {
    title: "Color Cube",
    description:
      "Material-coded reference cube used to demonstrate MTL-based shading in the viewer.",
    src: "/models/cube.obj",
    mtlSrc: "/models/cube.mtl",
  },
];

const systemArchitectureImages = [
  {
    title: "System Pipeline",
    description: "High-level pipeline from perception to actuation, showing how the system moves from detection to motion.",
    src: "/sys_arch/SystemPipeline.png",
  },
  {
    title: "Pick Sequence Flow",
    description: "Step-by-step sequence of a pick, from scanning the item to retracting the arm safely.",
    src: "/sys_arch/PickSequenceFlow.png",
  },
  {
    title: "Drop-off Routing Logic",
    description: "Routing logic that assigns items to specific drop-off locations based on AR tags.",
    src: "/sys_arch/Drop-offRoutingLogic.png",
  },
  {
    title: "Data Streams",
    description: "ROS 2 node graph showing how camera, detection, and control nodes exchange data.",
    src: "/sys_arch/DataStreams.png",
  },
  {
    title: "Topic & Service Summary",
    description: "Summary of the topics and services that connect the ROS 2 components in GroceryGizmo.",
    src: "/sys_arch/TopicAndServiceSummary.png",
  },
  {
    title: "TF Frame Hierarchy",
    description: "Transform tree illustrating the kinematic chain and AR marker frames.",
    src: "/sys_arch/TFFrameHierarchy.png",
  },
  {
    title: "Hardware Components",
    description: "Overview of the hardware setup connecting the control PC, sensors, and TM5-700 arm.",
    src: "/sys_arch/HardwareComponents.png",
  },
];

const teamContributions = [
  {
    name: "Arya Sasikumar",
    contributions: [
      "Integrated the TM5-700 workcell with the fridge fixtures and tuned approach trajectories for tight shelving.",
      "Modeled and fabricated the eye-in-hand RealSense mount and removable AR tag anchors for grocery items.",
    ],
  },
  {
    name: "Gursimar Virk",
    contributions: [
      "Led manipulation testing, including grasp offset calibration and safe approach sequences into the fridge.",
      "Developed safety guidelines for motion stop conditions, soft limits, and manual recovery procedures.",
    ],
  },
  {
    name: "Yamuna Rao",
    contributions: [
      "Built the perception stack for AR tag detection, TF broadcasting, and logic for selecting the nearest object.",
      "Benchmarked IK latency and tuned MoveIt2 request batching to reduce pick-and-place cycle time.",
    ],
  },
  {
    name: "Divya Krishnaswamy",
    contributions: [
      "Implemented the ROS 2 task manager that coordinates sensing, planning, and actuation for each grocery item.",
      "Profiled gripper force and validated grasp settings on fragile items to avoid bruising or slips.",
    ],
  },
  {
    name: "Patrick O’Connor",
    contributions: [
      "Built the Next.js project site, dashboards, and logging utilities used during demos.",
      "Maintained integration scripts between the TM driver, Modbus gripper control, and dashboard telemetry.",
    ],
  },
];

const hardwareComponents = [
  {
    name: "Omron TM5-700 Collaborative Arm",
    detail:
      "Six degree-of-freedom manipulator executing pick-and-place trajectories with a 700 mm reach and integrated safety monitoring.",
  },
  {
    name: "Robotiq 2F-85 Gripper",
    detail:
      "Modbus-controlled parallel gripper with configurable closure values that are tuned per AR tag class.",
  },
  {
    name: "Intel RealSense D435",
    detail:
      "Eye-in-hand RGB-D camera that provides aligned depth images for ArUco pose estimation in cluttered fridge scenes.",
  },
  {
    name: "Control PC (Ubuntu 22.04)",
    detail:
      "Runs ROS 2 Humble inside Distrobox, along with MoveIt2 planners, a Tkinter GUI, and trajectory logging tools.",
  },
  {
    name: "Custom AR Tag Fixtures",
    detail:
      "3D-printed mounts and laminated tags designed to withstand condensation while remaining easy for the camera to read.",
  },
];

const softwareModules = [
  {
    name: "Perception Pipeline",
    bullets: [
      "The Intel RealSense ROS 2 driver publishes synchronized RGB and depth frames on camera topics.",
      "A custom ArUco detector service finds markers and broadcasts marker frames through tf2.",
      "Camera-to-base calibration keeps transforms accurate so planning queries remain reliable.",
    ],
  },
  {
    name: "Task Manager",
    bullets: [
      "The robot_gui_recorder node keeps a queue of detected groceries, sorted by how easy they are to reach.",
      "It requests MoveIt2 /compute_ik for approach, grasp, and place poses with collision margins.",
      "It streams gripper commands and logs telemetry so runs can be replayed and debugged later.",
    ],
  },
  {
    name: "Motion Execution",
    bullets: [
      "A tm_driver wrapper sends joint-space trajectories with conservative velocities and soft joint limits.",
      "Fridge entry waypoints keep the elbow and wrist clear of the door and shelving edges.",
      "Exception handling pauses the cycle if TF frames drop out, the gripper times out, or unexpected torques are detected.",
    ],
  },
];

const systemWorkflow = [
  "The RealSense D435 publishes aligned RGB-D frames through the realsense2_camera_node.",
  "The ar_tag_detector_service finds grocery and shelf markers and publishes their transforms into tf2.",
  "The robot_gui_recorder node picks the closest valid grocery tag and pairs it with a fridge destination.",
  "MoveIt2 /compute_ik solves for approach, grasp, and placement joint targets while checking for collisions.",
  "The tm_driver executes the trajectory and logs joint states for later analysis.",
  "The Modbus gripper closes on the item, verifies force, and reopens after the item is placed.",
  "The system returns to the scan pose, updates which items remain, and repeats until the queue is empty.",
];

const implementationMedia = [
  {
    src: "/sys_arch/SystemPipeline.png",
    title: "Perception-to-actuation pipeline",
  },
  {
    src: "/sys_arch/TFFrameHierarchy.png",
    title: "TF frame hierarchy",
  },
  {
    src: "/sys_arch/HardwareComponents.png",
    title: "Hardware integration map",
  },
];

const resultHighlights = [
  {
    title: "Pick success rate",
    detail:
      "In final integrated dry runs, 8 out of 10 grocery items were placed correctly with door and shelf clearance checks.",
  },
  {
    title: "Average cycle time",
    detail:
      "The average time from detection to placement was about 72 seconds, mostly due to careful entry and exit motions in the fridge.",
  },
  {
    title: "System reliability",
    detail:
      "After adding approach offsets, no collisions were recorded. Two runs paused because of temporary TF dropouts that resolved on their own.",
  },
];

const resultMedia = [
  { src: "/arm2.jpg", title: "Arm reaching into the fridge" },
  { src: "/ARCo_detection.JPEG", title: "AR tag pose detection" },
  { src: "/ros.JPG", title: "ROS graph overview" },
];

const demoVideoUrl =
  "https://drive.google.com/file/d/1sSn59fT9UaXw09uU73KojV-ZqYzFssF2/preview";

const additionalMaterials = [
  {
    label: "Codebase",
    description: "Full Next.js site and project assets in the FinalProject/grocerygizmo repository.",
    href: "https://github.com/pchrisoc/GroceryGizmo/tree/main/FinalProject/grocerygizmo",
  },
  {
    label: "ROS 2 launch + nodes",
    description: "Perception, task orchestration, and TM driver wrappers described in FinalProject/sysarch.js.",
    href: "https://github.com/pchrisoc/GroceryGizmo/blob/main/FinalProject/sysarch.js",
  },
  {
    label: "CAD models",
    description: "3MF assets for camera mounts and workspace props, including ARCO.3mf, Cube1.3mf, Sphere.3mf, and more.",
    href: "https://github.com/pchrisoc/GroceryGizmo/tree/main/FinalProject",
  },
  {
    label: "Datasheets",
    description: "Hardware references for the Intel RealSense D435, Omron TM5-700, and Robotiq 2F-85.",
    href: "https://www.intelrealsense.com/depth-camera-d435/",
  },
  {
    label: "Demo media",
    description: "Additional photos and videos archived in the project drive for grading and outreach.",
    href: demoVideoUrl,
  },
];

const rosCodeSnippet = String.raw`# Example ROS 2 pipeline for GroceryGizmo

# Camera node publishes RGB(+depth)
#   /camera/image_raw

# AR Code node → detects markers
#   /aruco/poses
#   /tf

# Pose Estimator → outputs PoseStamped in camera frame
#   /grocery_pose

# Object Selector:
#   - picks nearest target
#   - transforms pose to base_link
#   - publishes:
#       /target_pose

# MoveIt + IK:
#   - generate collision-free pick+place trajectories
#   - adjust gripping force based on AR tag class
`;

export default function Page() {
  const [collapsed, setCollapsed] = React.useState(false);
  const [preview, setPreview] = React.useState<
    (typeof systemArchitectureImages)[number] | null
  >(null);
  const [selectedCodeFile, setSelectedCodeFile] = React.useState(codeSamples[0].value);
  const [mobileNavOpen, setMobileNavOpen] = React.useState(false);
  const [expandedMember, setExpandedMember] = React.useState<string | null>(null);

  const selectedSample = React.useMemo(
    () => codeSamples.find((sample) => sample.value === selectedCodeFile) ?? codeSamples[0],
    [selectedCodeFile]
  );

  const codeTheme = React.useMemo(
    () => CODE_THEME_MAP[selectedSample.language] ?? CODE_THEME_MAP.default,
    [selectedSample.language]
  );

  const handleCodeChange = (event: SelectChangeEvent<string>) => {
    setSelectedCodeFile(event.target.value as string);
  };

  React.useEffect(() => {
    const handleScroll = () => {
      setCollapsed(window.scrollY > 110);
    };

    handleScroll();
    window.addEventListener("scroll", handleScroll, { passive: true });
    return () => window.removeEventListener("scroll", handleScroll);
  }, []);

  const navButtonSx = React.useMemo(
    () => ({
      fontSize: collapsed ? "0.85rem" : "0.95rem",
      textTransform: "none",
      px: collapsed ? 1 : 1.5,
      transition: "font-size 0.3s ease, color 0.3s ease, padding 0.3s ease",
    }),
    [collapsed]
  );

  const scrollTo = React.useCallback((id: string) => {
    const el = document.getElementById(id);
    if (el) el.scrollIntoView({ behavior: "smooth" });
  }, []);

  const handleMobileNavSelect = (id: string) => {
    setMobileNavOpen(false);
    scrollTo(id);
  };

  const toggleMemberDescription = React.useCallback((name: string) => {
    setExpandedMember((prev) => (prev === name ? null : name));
  }, []);

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />

      {/* NAVBAR */}
      <AppBar
        position="sticky"
        elevation={collapsed ? 6 : 0}
        color="transparent"
        sx={{
          transition: "all 0.3s ease",
          backgroundColor: collapsed ? "rgba(9,11,25,0.92)" : "transparent",
          backdropFilter: collapsed ? "blur(12px)" : "none",
          borderBottom: collapsed ? "1px solid rgba(255,255,255,0.08)" : "none",
        }}
      >
        <Toolbar
          sx={{
            transition: "all 0.3s ease",
            minHeight: collapsed ? 56 : 88,
            py: collapsed ? 0.5 : 2,
          }}
        >
          <Typography
            variant="h6"
            sx={{
              flexGrow: 1,
              fontWeight: 600,
              cursor: "pointer",
              transition: "font-size 0.3s ease",
              fontSize: collapsed ? "1.05rem" : "1.25rem",
            }}
            onClick={() => scrollTo("hero")}
          >
            GroceryGizmo
          </Typography>

          <Box
            sx={{
              display: { xs: "none", md: "flex" },
              gap: collapsed ? 1.5 : 2,
              transition: "gap 0.3s ease",
            }}
          >
            {NAV_SECTIONS.map((section) => (
              <Button
                key={section.id}
                color="inherit"
                onClick={() => scrollTo(section.id)}
                sx={navButtonSx}
              >
                {section.label}
              </Button>
            ))}
          </Box>

          <IconButton
            sx={{
              display: { xs: "flex", md: "none" },
              transition: "all 0.3s ease",
              border: collapsed ? "1px solid rgba(255,255,255,0.2)" : "none",
              borderRadius: 2,
              p: collapsed ? 0.5 : 1,
            }}
            aria-label="Open navigation"
            onClick={() => setMobileNavOpen(true)}
          >
            <MenuIcon />
          </IconButton>
        </Toolbar>
      </AppBar>

      <Drawer
        anchor="right"
        open={mobileNavOpen}
        onClose={() => setMobileNavOpen(false)}
        ModalProps={{ keepMounted: true }}
        PaperProps={{
          sx: {
            width: "80vw",
            maxWidth: 320,
            background: "rgba(9,11,25,0.96)",
            color: "grey.100",
            backdropFilter: "blur(16px)",
            borderLeft: "1px solid rgba(255,255,255,0.08)",
          },
        }}
      >
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            justifyContent: "space-between",
            px: 2.5,
            py: 2,
          }}
        >
          <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
            Navigate
          </Typography>
          <IconButton
            aria-label="Close navigation"
            onClick={() => setMobileNavOpen(false)}
            sx={{ color: "grey.300" }}
          >
            <CloseIcon />
          </IconButton>
        </Box>
        <Divider sx={{ borderColor: "rgba(255,255,255,0.08)" }} />
        <List sx={{ p: 0 }}>
          {NAV_SECTIONS.map((section) => (
            <ListItem key={section.id} disablePadding>
              <ListItemButton
                onClick={() => handleMobileNavSelect(section.id)}
                sx={{
                  px: 2.5,
                  py: 1.5,
                  color: "grey.100",
                  "&:hover": {
                    backgroundColor: "rgba(127,255,212,0.08)",
                    color: "#7fffd4",
                  },
                }}
              >
                <ListItemText
                  primary={section.label}
                  primaryTypographyProps={{
                    sx: { fontWeight: 500, letterSpacing: 0.2 },
                  }}
                />
              </ListItemButton>
            </ListItem>
          ))}
        </List>
      </Drawer>

      {/* =========================================== */}
      {/* HERO SECTION */}
      {/* =========================================== */}

      <Box
        id="hero"
        sx={{
          minHeight: "70vh",
          display: "flex",
          alignItems: "center",
          py: 8,
          background:
            "radial-gradient(circle at top, rgba(127,255,212,0.12), transparent 50%)",
        }}
      >
        <Container maxWidth="lg">
          <Grid container spacing={6} alignItems="center">
            {/* LEFT SIDE */}
            <Grid size={{ xs: 12, md: 7 }}>
              <Typography
                variant="overline"
                sx={{ letterSpacing: 2, color: "primary.main" }}
              >
                EECS 106A Robotics Final Project
              </Typography>

              <Typography variant="h3" sx={{ fontWeight: 700, mt: 1, mb: 2 }}>
                GroceryGizmo
                <br />
                <Box component="span" sx={{ color: "primary.main", fontSize: "0.8em" }}>
                  Robot Grocery Assistant
                </Box>
              </Typography>

              <Typography variant="body1" sx={{ mb: 3, color: "grey.300" }}>
                A six-degree-of-freedom Omron robot arm that uses AR tags to recognize groceries and autonomously load them into a real refrigerator.
              </Typography>

              <Box sx={{ display: "flex", gap: 2, flexWrap: "wrap" }}>
                <Button
                  variant="contained"
                  size="large"
                  onClick={() => scrollTo("project")}
                >
                  View Project Details
                </Button>
                <Button
                  variant="outlined"
                  size="large"
                  onClick={() => scrollTo("gallery")}
                >
                  View Gallery
                </Button>
              </Box>
            </Grid>

            {/* RIGHT SIDE */}
            <Grid size={{ xs: 12, md: 5 }}>
              <ScrollReactiveCard
                accentKey="hero-omron-card"
                effectVariant="pulse"
                sx={{
                  borderRadius: 3,
                  overflow: "hidden",
                  backdropFilter: "blur(14px)",
                  border: "1px solid rgba(127,255,212,0.25)",
                  background: "linear-gradient(160deg, rgba(9,11,25,0.92), rgba(17,20,38,0.92))",
                  boxShadow: "0 24px 60px rgba(9,11,25,0.45)",
                }}
              >
                <Box
                  sx={{
                    position: "relative",
                    height: 320,
                    background: "radial-gradient(circle at top, rgba(127,255,212,0.25), transparent 55%)",
                  }}
                >
                  <ObjViewer
                    src="/models/TM5A-700-OMRON.obj"
                    mtlSrc="/models/TM5A-700-OMRON.mtl"
                    height={320}
                    background="transparent"
                    fallbackColor="#7fffd4"
                    autoRotateAxis="z"
                    autoRotateSpeed={0.25}
                  />
                </Box>
                <CardContent sx={{ display: "flex", flexDirection: "column", gap: 1 }}>
                  <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                    Omron TM5-700 with Robotiq Gripper
                  </Typography>
                  <Typography variant="body2" sx={{ color: "grey.400", lineHeight: 1.6 }}>
                    You can drag, pinch, and orbit the full TM5 model to see the workspace clearances and wrist-mounted RealSense setup directly in the browser.
                  </Typography>
                </CardContent>
              </ScrollReactiveCard>
            </Grid>
          </Grid>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* INTRODUCTION */}
      {/* =========================================== */}

      <Box id="introduction" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            Introduction
          </Typography>

          <ScrollReactiveCard
            accentKey="introduction-problem"
            effectVariant="lift"
            sx={{
              borderRadius: 3,
              border: "1px solid rgba(255,255,255,0.08)",
              mb: 4,
            }}
          >
            <CardContent sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
              <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                The Problem
              </Typography>
              <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                For a lot of people, putting groceries into the fridge is more than just a small chore. It can be physically demanding and uncomfortable, especially when items are heavy, shelves are deep, or space is tight. Reaching high shelves, bending down over and over, and twisting around open doors can be hard or unsafe for older adults, people with limited mobility, shorter users, or anyone dealing with fatigue or injury.
              </Typography>
              <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                On top of that, there is a constant mental task of figuring out where everything should go in a crowded, messy fridge. Robots do not get tired or frustrated by cramped spaces, but teaching a robot to understand those spaces and move safely inside them is a real challenge.
              </Typography>
            </CardContent>
          </ScrollReactiveCard>
          <Grid container spacing={3}>
            <Grid size={{ xs: 12, md: 6 }}>
              <ScrollReactiveCard
                accentKey="introduction-solution"
                effectVariant="slide"
                sx={{
                  borderRadius: 3,
                  border: "1px solid rgba(255,255,255,0.08)",
                  height: "100%",
                }}
              >
                <CardContent sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
                  <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                    Our Solution
                  </Typography>
                  <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                    GroceryGizmo is an autonomous fridge-loading system that uses AR tags, computer vision, and a six-degree-of-freedom Omron arm to handle both perception and manipulation:
                  </Typography>
                  <Box component="ul" sx={{ pl: 3, color: "grey.200", lineHeight: 1.7, m: 0 }}>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Sensing: The camera detects AR-tagged grocery items on the counter and AR-tagged slots inside the fridge.
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Planning: The system picks the closest item, decides where it should go, and computes a safe pose in the robot’s base frame.
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Actuation: The arm picks up each item and places it in its assigned location, even in tight, awkward spaces inside the fridge.
                      </Typography>
                    </li>
                  </Box>
                  <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                    The result is a full pipeline that shows how a robot can handle a real household task from start to finish.
                  </Typography>
                </CardContent>
              </ScrollReactiveCard>
            </Grid>
            <Grid size={{ xs: 12, md: 6 }}>
              <ScrollReactiveCard
                accentKey="introduction-impact"
                effectVariant="tilt"
                sx={{
                  borderRadius: 3,
                  border: "1px solid rgba(255,255,255,0.08)",
                  height: "100%",
                }}
              >
                <CardContent sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
                  <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                    The Impact
                  </Typography>
                  <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                    As AI and robotics become more common, we want them to support people in everyday life, not just in factories. The goal is not to replace meaningful work, but to offload tasks that are repetitive, physically demanding, or simply out of reach for many people.
                  </Typography>
                  <Box component="ul" sx={{ pl: 3, color: "grey.200", lineHeight: 1.7, m: 0 }}>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Older adults can stay independent longer.
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        People with limited mobility avoid risky bending, lifting, and twisting.
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Busy households can hand off a small but constant source of work.
                      </Typography>
                    </li>
                  </Box>
                  <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                    GroceryGizmo is one step toward assistive home robots that quietly take care of the small, tiring tasks in the background.
                  </Typography>
                </CardContent>
              </ScrollReactiveCard>
            </Grid>
          </Grid>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* TEAM SECTION */}
      {/* =========================================== */}

      <Box id="team" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 1 }}>
            Team
          </Typography>
          <Typography sx={{ color: "grey.400", mb: 4 }}>
            A multidisciplinary team from EECS, Mechanical Engineering, Bioengineering, and Business.
          </Typography>

          <Grid container spacing={3}>
            {teamMembers.map((m, index) => {
              const isExpanded = expandedMember === m.name;
              const accentColor = accentFromKey(`team-${m.name}`);
              const accentGlow = hexToRgba(accentColor, 0.25);
              const mobileOverlayBase = getMobileOverlayStyles(accentColor);

              return (
                <Grid size={{ xs: 12, md: 6 }} key={m.name}>
                  <ScrollReactiveCard
                    accentKey={`team-${m.name}`}
                    effectVariant={REACTIVE_VARIANTS[index % REACTIVE_VARIANTS.length]}
                    sx={{
                      borderRadius: 3,
                      border: "1px solid rgba(255,255,255,0.08)",
                      overflow: "hidden",
                    }}
                  >
                    <CardContent sx={{ p: 0, "&:last-child": { pb: 0 } }}>
                      <Box
                        sx={{
                          display: { xs: "none", md: "flex" },
                          alignItems: "stretch",
                        }}
                      >
                        <Avatar
                          variant="square"
                          alt={m.name}
                          src={m.avatar}
                          sx={{
                            width: 200,
                            height: 200,
                            flexShrink: 0,
                            borderRadius: 0,
                            bgcolor: m.avatar ? "transparent" : stringToAvatarColor(m.name),
                            color: m.avatar ? undefined : "rgba(9,11,19,0.92)",
                            fontSize: 18,
                            fontWeight: 600,
                            "& img": {
                              objectFit: "cover",
                            },
                          }}
                        >
                          {initialsFromName(m.name)}
                        </Avatar>

                        <Box
                          sx={{
                            flex: 1,
                            p: 3,
                            display: "flex",
                            flexDirection: "column",
                            justifyContent: "center",
                            gap: 1,
                          }}
                        >
                          <Typography variant="h6" sx={{ fontWeight: 600 }}>
                            {m.name}
                          </Typography>
                          <Typography
                            variant="subtitle2"
                            sx={{ fontStyle: "italic", color: "grey.300" }}
                          >
                            {m.role}
                          </Typography>
                          <Typography variant="body2" sx={{ color: "grey.400", lineHeight: 1.6 }}>
                            {m.background}
                          </Typography>
                        </Box>
                      </Box>

                      <Box
                        sx={{
                          display: { xs: "block", md: "none" },
                          position: "relative",
                        }}
                      >
                        <Avatar
                          variant="square"
                          alt={m.name}
                          src={m.avatar}
                          sx={{
                            width: "100%",
                            height: 280,
                            borderRadius: 0,
                            bgcolor: m.avatar ? "transparent" : stringToAvatarColor(m.name),
                            color: m.avatar ? undefined : "rgba(9,11,19,0.92)",
                            fontSize: 22,
                            fontWeight: 600,
                            "& img": {
                              objectFit: "cover",
                              objectPosition: "center",
                            },
                          }}
                        >
                          {initialsFromName(m.name)}
                        </Avatar>

                        <Box
                          sx={{
                            position: "absolute",
                            inset: 0,
                            pointerEvents: "none",
                            background: `linear-gradient(180deg, transparent 45%, rgba(5,7,17,0.88) 100%)`,
                            zIndex: 1,
                          }}
                        />

                        <Box
                          sx={{
                            position: "absolute",
                            left: 0,
                            right: 0,
                            bottom: 0,
                            p: 2.5,
                            display: "flex",
                            flexDirection: "column",
                            gap: 0.5,
                            zIndex: 2,
                          }}
                        >
                          <Typography variant="h6" sx={{ fontWeight: 600 }}>
                            {m.name}
                          </Typography>
                          <Typography
                            variant="subtitle2"
                            sx={{ fontStyle: "italic", color: "rgba(255,255,255,0.78)" }}
                          >
                            {m.role}
                          </Typography>
                        </Box>

                        <IconButton
                          aria-label={isExpanded ? `Hide ${m.name} bio` : `Show ${m.name} bio`}
                          onClick={() => toggleMemberDescription(m.name)}
                          sx={{
                            position: "absolute",
                            top: 14,
                            right: 14,
                            color: accentColor,
                            backgroundColor: "rgba(9,11,25,0.7)",
                            border: `1px solid ${hexToRgba(accentColor, 0.4)}`,
                            boxShadow: isExpanded
                              ? `0 12px 28px ${accentGlow}`
                              : `0 6px 18px rgba(5,7,17,0.45)`,
                            transition: "transform 0.3s ease, box-shadow 0.3s ease, background-color 0.3s ease",
                            transform: isExpanded ? "rotate(45deg)" : "rotate(0deg)",
                            "&:hover": {
                              backgroundColor: "rgba(9,11,25,0.86)",
                            },
                            zIndex: 3,
                          }}
                        >
                          <AddIcon />
                        </IconButton>

                        <Box
                          role="dialog"
                          aria-modal={isExpanded}
                          aria-hidden={!isExpanded}
                          onClick={() => toggleMemberDescription(m.name)}
                          sx={isExpanded
                            ? {
                                ...mobileOverlayBase,
                                opacity: 1,
                                pointerEvents: "auto",
                                transform: "translateY(0)",
                                boxShadow: `0 22px 48px ${accentGlow}`,
                              }
                            : mobileOverlayBase
                          }
                        >
                          <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                            {m.name}
                          </Typography>
                          <Typography variant="body2" sx={{ lineHeight: 1.7 }}>
                            {m.background}
                          </Typography>
                          <Typography
                            variant="caption"
                            sx={{
                              textTransform: "uppercase",
                              letterSpacing: 1,
                              color: "rgba(255,255,255,0.72)",
                            }}
                          >
                            Tap anywhere to close
                          </Typography>
                        </Box>
                      </Box>
                    </CardContent>
                  </ScrollReactiveCard>
                </Grid>
              );
            })}
          </Grid>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* DESIGN */}
      {/* =========================================== */}

      <Box id="design" sx={{ py: 8 }}>

        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            Design
          </Typography>

          <ScrollReactiveCard
            accentKey="design-criteria"
            effectVariant="slide"
            sx={{
              borderRadius: 3,
              border: "1px solid rgba(255,255,255,0.08)",
              mb: 4,
            }}
          >
            <CardContent sx={{ display: "flex", flexDirection: "column", gap: 2 }}>

              <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                Our Design Criteria
              </Typography>

              <Box sx={{ display: "flex", flexDirection: "column", gap: 1.5 }}>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Detect grocery items using AR tags
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    The robot must reliably detect AR-tagged objects on the counter and estimate their 3D positions, regardless of how each item is oriented.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Identify placement locations inside the fridge
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    AR tags inside the refrigerator act as labeled anchor points, telling the robot exactly where each item should be placed.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Determine which item to pick first
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    The system should automatically pick the closest reachable item to reduce motion and keep the workflow efficient.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Transform camera detections into robot coordinates
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    Accurate coordinate transforms are crucial so the Omron arm can move to the correct pre-grasp and placement poses in its own base frame.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Grasp and place items safely
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    The robot should approach with a safe offset, lower the gripper for a stable grasp, move through the fridge without collisions, and place items gently at their tagged locations.
                  </Typography>
                </Box>
              </Box>
            </CardContent>
          </ScrollReactiveCard>

          <ScrollReactiveCard
            accentKey="design-functionality"
            effectVariant="tilt"
            sx={{
              borderRadius: 3,
              border: "1px solid rgba(255,255,255,0.08)",
              mb: 4,
            }}
          >
            <CardContent sx={{ display: "flex", flexDirection: "column", gap: 2 }}>

              <Typography variant="subtitle1" sx={{ fontWeight: 600, mt: 2 }}>
                Desired Functionality
              </Typography>
              <Box component="ul" sx={{ pl: 3, color: "grey.200", lineHeight: 1.7, m: 0 }}>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Perceive items on a counter.
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Decide where each item belongs inside the fridge.
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Plan reachable poses for grasping and placing each item.
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Move the arm safely through the constrained fridge space.
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Place groceries accurately and gently in their intended spots.
                  </Typography>
                </li>
              </Box>
              <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                Together, these criteria guide GroceryGizmo toward being predictable, safe, and actually helpful in a real kitchen setting instead of just a lab environment.
              </Typography>
            </CardContent>
          </ScrollReactiveCard>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* IMPLEMENTATION */}
      {/* =========================================== */}

      <Box id="implementation" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            Implementation
          </Typography>
          <Typography sx={{ color: "grey.300", mb: 4, lineHeight: 1.7 }}>
            The GroceryGizmo stack combines collaborative robot hardware with ROS 2, MoveIt2 planning, and a lightweight GUI. Each grocery item is scanned on the counter, matched to a location in the fridge, and moved along a planned trajectory that respects the door, shelves, and workspace limits.
          </Typography>
          <Typography variant="h6" sx={{ fontWeight: 700, mb: 2 }}>
            System Architecture
          </Typography>
          <Typography sx={{ color: "grey.400", mb: 4 }}>
            Expand each diagram to explore how perception, planning, and hardware all connect in GroceryGizmo.
          </Typography>

          <Grid container spacing={3}>
            {systemArchitectureImages.map((item, index) => (
              <Grid key={item.title} size={{ xs: 12, md: 6, lg: 4 }}>
                <ScrollReactiveCard
                  accentKey={`system-architecture-${item.title}`}
                  effectVariant={REACTIVE_VARIANTS[index % REACTIVE_VARIANTS.length]}
                  sx={{
                    borderRadius: 3,
                    border: "1px solid rgba(255,255,255,0.08)",
                    height: "100%",
                    display: "flex",
                  }}
                >
                  <CardActionArea
                    onClick={() => setPreview(item)}
                    sx={{
                      display: "flex",
                      flexDirection: "column",
                      alignItems: "stretch",
                      height: "100%",
                    }}
                  >
                    <Box
                      sx={{
                        position: "relative",
                        width: "100%",
                        height: 220,
                        borderBottom: "1px solid rgba(255,255,255,0.08)",
                        overflow: "hidden",
                      }}
                    >
                      <Image
                        src={item.src}
                        alt={item.title}
                        fill
                        sizes="(max-width: 1024px) 100vw, 50vw"
                        style={{ objectFit: "cover" }}
                      />
                    </Box>
                    <CardContent sx={{ flexGrow: 1, width: "100%" }}>
                      <Typography variant="h6" sx={{ mb: 0.5 }}>
                        {item.title}
                      </Typography>
                      <Typography variant="body2" sx={{ color: "grey.400" }}>
                        {item.description}
                      </Typography>
                    </CardContent>
                  </CardActionArea>
                </ScrollReactiveCard>
              </Grid>
            ))}
          </Grid>
        </Container>
      </Box>

      <Divider />

          {/* =========================================== */}
          {/* CODE SECTION */}
          {/* =========================================== */}

          <Box id="code" sx={{ py: 8 }}>
            <Container maxWidth="lg">
              <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
                Code
              </Typography>
              <Typography sx={{ color: "grey.400", mb: 4 }}>
                Take a look at a few representative snippets that launch perception, refresh dashboards, and deploy GroceryGizmo with a single command.
              </Typography>

              <Grid container spacing={4} alignItems="stretch">
                <Grid size={{ xs: 12, md: 5 }}>
                  <ScrollReactiveCard
                    accentKey="code-controls"
                    effectVariant="slide"
                    sx={{
                      borderRadius: 3,
                      border: "1px solid rgba(255,255,255,0.08)",
                      height: "100%",
                    }}
                  >
                    <CardContent sx={{ display: "flex", flexDirection: "column", gap: 3 }}>
                      <Box>
                        <Typography variant="subtitle2" sx={{ color: "grey.200", textTransform: "uppercase", letterSpacing: 1.5 }}>
                          Code Library
                        </Typography>
                        <Typography variant="h6" sx={{ fontWeight: 600, mt: 1 }}>
                          Select a file to explore
                        </Typography>
                        <Typography sx={{ color: "grey.300", mt: 1.5, lineHeight: 1.7 }}>
                          These three files illustrate key parts of the system: the ROS launch file, a web API route for cache invalidation, and a deployment helper script.
                        </Typography>
                      </Box>

                      <FormControl fullWidth variant="outlined" sx={{ "& .MuiInputLabel-root": { color: "grey.400" }, "& .MuiOutlinedInput-notchedOutline": { borderColor: "rgba(255,255,255,0.12)" } }}>
                        <InputLabel id="code-sample-label">Code sample</InputLabel>
                        <Select
                          labelId="code-sample-label"
                          id="code-sample"
                          value={selectedCodeFile}
                          label="Code sample"
                          onChange={handleCodeChange}
                          sx={{ color: "grey.100" }}
                        >
                          {codeSamples.map((sample) => (
                            <MenuItem key={sample.value} value={sample.value}>
                              {sample.label}
                            </MenuItem>
                          ))}
                        </Select>
                      </FormControl>

                      <Box sx={{ display: "flex", flexDirection: "column", gap: 1 }}>
                        <Typography variant="subtitle2" sx={{ color: "grey.200", fontWeight: 600 }}>
                          What you&apos;re viewing
                        </Typography>
                        <Typography sx={{ color: "grey.400", lineHeight: 1.6 }}>
                          {selectedSample.description}
                        </Typography>
                      </Box>
                    </CardContent>
                  </ScrollReactiveCard>
                </Grid>

                <Grid size={{ xs: 12, md: 7 }}>
                  <ScrollReactiveCard
                    accentKey={`code-${selectedSample.language}`}
                    accentColor={codeTheme.badgeText}
                    effectVariant="prism"
                    sx={{
                      height: "100%",
                      borderRadius: 3,
                      border: `1px solid ${codeTheme.border}`,
                      boxShadow: codeTheme.glow,
                      background: codeTheme.background,
                      color: codeTheme.text,
                    }}
                  >
                    <CardContent sx={{ display: "flex", flexDirection: "column", gap: 3, height: "100%" }}>
                      <Box sx={{ display: "flex", flexWrap: "wrap", justifyContent: "space-between", alignItems: "center", gap: 1.5 }}>
                        <Typography variant="h6" sx={{ fontWeight: 600 }}>
                          {selectedSample.label}
                        </Typography>
                        <Box
                          sx={{
                            px: 1.5,
                            py: 0.5,
                            borderRadius: 1.5,
                            fontSize: "0.75rem",
                            textTransform: "uppercase",
                            letterSpacing: 1.2,
                            background: codeTheme.badge,
                            color: codeTheme.badgeText,
                            border: `1px solid ${codeTheme.border}`,
                          }}
                        >
                          {selectedSample.language}
                        </Box>
                      </Box>

                      <Box
                        component="pre"
                        sx={{
                          flex: 1,
                          m: 0,
                          p: 2.5,
                          borderRadius: 2,
                          overflow: "auto",
                          fontFamily: "'Fira Code', 'JetBrains Mono', monospace",
                          fontSize: "0.9rem",
                          lineHeight: 1.6,
                          background: "rgba(0,0,0,0.22)",
                          border: "1px solid rgba(255,255,255,0.08)",
                          color: codeTheme.text,
                        }}
                      >
                        {selectedSample.code}
                      </Box>
                    </CardContent>
                  </ScrollReactiveCard>
                </Grid>
              </Grid>
            </Container>
          </Box>

          <Divider />

      {/* =========================================== */}
      {/* 3D MODELS GALLERY */}
      {/* =========================================== */}

      <Box id="viewer" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            3D Models Gallery
          </Typography>
          <Typography sx={{ color: "grey.400", mb: 4 }}>
            Explore the simplified CAD assets we use to plan the workspace layout, storage locations, and reachability inside the fridge.
          </Typography>

          <Grid container spacing={4}>
            {modelGallery.map((model, index) => (
              <Grid size={{ xs: 12, md: 4 }} key={model.title}>
                <ScrollReactiveCard
                  accentKey={`model-gallery-${model.title}`}
                  effectVariant={REACTIVE_VARIANTS[index % REACTIVE_VARIANTS.length]}
                  sx={{
                    borderRadius: 3,
                    border: "1px solid rgba(255,255,255,0.12)",
                    display: "flex",
                    flexDirection: "column",
                    height: "100%",
                    overflow: "hidden",
                  }}
                >
                  <Box
                    sx={{
                      borderBottom: "1px solid rgba(255,255,255,0.08)",
                      background: "rgba(255,255,255,0.02)",
                    }}
                  >
                    <ObjViewer
                      src={model.src}
                      mtlSrc={model.mtlSrc}
                      height={240}
                      autoRotateAxis="z"
                      autoRotateSpeed={0.25}
                    />
                  </Box>

                  <CardContent>
                    <Typography variant="h6" sx={{ mb: 1 }}>
                      {model.title}
                    </Typography>
                    <Typography variant="body2" sx={{ color: "grey.400" }}>
                      {model.description}
                    </Typography>
                  </CardContent>
                </ScrollReactiveCard>
              </Grid>
            ))}
          </Grid>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* RESULTS */}
      {/* =========================================== */}

      <Box id="results" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            Results
          </Typography>
          <Typography sx={{ color: "grey.300", mb: 4, lineHeight: 1.7 }}>
            We evaluated GroceryGizmo on a full countertop-to-fridge workflow using AR-tagged groceries of different shapes and sizes. The system executed autonomous pick-and-place cycles, logged motion telemetry, and demonstrated how perception, planning, and manipulation interact in a real setup.
          </Typography>
          <ScrollReactiveCard
            accentKey="results-demo"
            effectVariant="ripple"
            sx={{
              borderRadius: 3,
              border: "1px solid rgba(255,255,255,0.08)",
            }}
          >
            <CardContent sx={{ display: "flex", flexDirection: "column", gap: 1 }}>
              <Box
                component="iframe"
                src="https://drive.google.com/file/d/1sSn59fT9UaXw09uU73KojV-ZqYzFssF2/preview"
                title="GroceryGizmo Demo Video"
                allow="autoplay; fullscreen; picture-in-picture"
                allowFullScreen
                sx={{
                  width: "100%",
                  aspectRatio: "16 / 9",
                  borderRadius: 2,
                  border: "1px solid rgba(255,255,255,0.12)",
                  backgroundColor: "rgba(255,255,255,0.02)",
                  mt: 2,
                  boxShadow: "0 18px 36px rgba(0,0,0,0.35)",
                }}
              />
            </CardContent>
          </ScrollReactiveCard>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* CONCLUSION */}
      {/* =========================================== */}

      <Box id="conclusion" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            Conclusion
          </Typography>
          <Typography sx={{ color: "grey.300", mb: 3, lineHeight: 1.7 }}>
            GroceryGizmo met its main design goals: it could detect groceries, match them to fridge placements, and execute constrained pick-and-place motions. The robot handled several item types without collisions and maintained consistent localization accuracy across repeated runs.
          </Typography>

          <Typography variant="h6" sx={{ fontWeight: 600, mb: 1.5 }}>
            Challenges
          </Typography>
          <Box component="ul" sx={{ pl: 3, color: "grey.300", lineHeight: 1.7, mb: 3 }}>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Dim or reflective light prevented stable AR code detection.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Coordinating entry angles into the fridge with a wrist-mounted camera took several rounds of offset calibration.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Lining up Robotiq gripper with the edges of each unique object proved difficult.
              </Typography>
            </li>
          </Box>

          <Typography variant="h6" sx={{ fontWeight: 600, mb: 1.5 }}>
            Known Limitations & Future Work
          </Typography>
          <Box component="ul" sx={{ pl: 3, color: "grey.300", lineHeight: 1.7 }}>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                The current system depends on AR tags for both groceries and fridge locations. An obvious next step is using vision-based item recognition and shelf detection instead of printed markers.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Our workflow ends with the fridge door open. Adding a closing door manipulation would make the system more fully autonomous.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Integrating tactile sensing in the gripper could further reduce slip risk and improve handling of heavier or softer items.
              </Typography>
            </li>
          </Box>
        </Container>
      </Box>

      <Divider />

      {/* FOOTER */}
      <Box sx={{ py: 4 }}>
        <Container maxWidth="lg">
          <Typography
            variant="body2"
            sx={{ color: "grey.500", textAlign: "center" }}
          >
            GroceryGizmo • EECS 106A •{" "}
            <MuiLink
              href="https://eecs.berkeley.edu"
              target="_blank"
              rel="noopener noreferrer"
              underline="hover"
              color="inherit"
            >
              UC Berkeley
            </MuiLink>
          </Typography>
        </Container>
      </Box>

      <Dialog
        fullScreen
        open={Boolean(preview)}
        onClose={() => setPreview(null)}
        PaperProps={{ sx: { backgroundColor: "rgba(5,7,17,0.95)" } }}
      >
        {preview && (
          <Box
            sx={{
              display: "flex",
              flexDirection: "column",
              height: "100%",
              background: "radial-gradient(circle at top, rgba(127,255,212,0.08), transparent 65%)",
            }}
          >
            <Box
              sx={{
                display: "flex",
                alignItems: "center",
                justifyContent: "space-between",
                px: { xs: 2, md: 4 },
                py: 2,
                borderBottom: "1px solid rgba(255,255,255,0.08)",
              }}
            >
              <Box>
                <Typography variant="h5" sx={{ fontWeight: 600 }}>
                  {preview.title}
                </Typography>
                <Typography variant="body2" sx={{ color: "grey.400", mt: 0.5 }}>
                  {preview.description}
                </Typography>
              </Box>
              <IconButton onClick={() => setPreview(null)}>
                <CloseIcon />
              </IconButton>
            </Box>

            <DialogContent
              sx={{
                flex: 1,
                display: "flex",
                minHeight: 0,
                justifyContent: "center",
                alignItems: "center",
                p: { xs: 2, md: 4 },
              }}
            >
              <Box
                sx={{
                  position: "relative",
                  width: "100%",
                  height: "100%",
                  minHeight: 0,
                  borderRadius: 3,
                  overflow: "hidden",
                  border: "1px solid rgba(255,255,255,0.12)",
                }}
              >
                <Image
                  src={preview.src}
                  alt={preview.title}
                  fill
                  sizes="100vw"
                  style={{ objectFit: "contain" }}
                />
              </Box>
            </DialogContent>
          </Box>
        )}
      </Dialog>
    </ThemeProvider>
  );
}
