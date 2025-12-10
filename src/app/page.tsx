"use client";

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
  Dialog,
  DialogContent,
  Divider,
  IconButton,
  ImageList,
  ImageListItem,
  Link as MuiLink,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  Toolbar,
  Typography,
  Grid,
} from "@mui/material";
import MenuIcon from "@mui/icons-material/Menu";
import CloseIcon from "@mui/icons-material/Close";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import Image from "next/image";
import dynamic from "next/dynamic";
import type { ObjViewerProps } from "./components/ObjViewer";

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

type ScrollReactiveCardProps = React.ComponentProps<typeof Card> & {
  accentKey: string;
  accentColor?: string;
  variant?: ReactiveVariant;
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
  variant,
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
    () => variant ?? variantFromKey(accentKey),
    [variant, accentKey]
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
  background: string;
  avatar?: string;
}> = [
  {
    name: "Arya Sasikumar",
    background:
      "Mechanical Engineering & Business major interested in robotics. Well-versed in Solidworks, ROS, Python, and C++.",
    avatar: "/arya.jpeg",
  },
  {
    name: "Gursimar Virk",
    background:
      "Mechanical Engineering major and Material Science minor interested in robotics. Works with humanoids; skilled in Solidworks, ROS, Python, and C++.",
    avatar: "/gursimar.jpeg",
  },
  {
    name: "Yamuna Rao",
    background:
      "EECS major interested in robotics/optimization. Experience with CV, Python, C, PyTorch, and TensorFlow.",
    avatar: "/yamuna.jpeg",
  },
  {
    name: "Divya Krishnaswamy",
    background:
      "Bioengineering major & EECS minor interested in surgical robotics. Experience with Solidworks, C++, Python, and embedded systems.",
    avatar: "/divya.jpeg",
  },
  {
    name: "Patrick O’Connor",
    background:
      "Third-year EECS major with interest in EE & robotics. Experience with Fusion, KiCad, and Python.",
    avatar: "/patrick.jpg",
  },
];

const galleryItems = [
  { src: "/arm.jpg", title: "Omron arm ready for pick" },
  { src: "/arm2.jpg", title: "Arm reaching into fridge" },
  { src: "/arm3.JPG", title: "Close-up of gripper" },
  { src: "/ARCo_detection.JPEG", title: "AR tag pose detection" },
  { src: "/object_detection.JPEG", title: "Object detection pipeline" },
  { src: "/ros.JPG", title: "ROS graph overview" },
];

const modelGallery = [
  {
    title: "Camera Mount",
    description:
      "Custom Intel Realsense mount for Omron robotic arm.",
    src: "/models/mount.obj",
    mtlSrc: "/models/mount.mtl",
  },

  {
    title: "Color Cube",
    description:
      "Material-coded reference cube demonstrating MTL-driven shading in the viewer.",
    src: "/models/cube.obj",
    mtlSrc: "/models/cube.mtl",
  },
];

const systemArchitectureImages = [
  {
    title: "System Pipeline",
    description: "High-level AR pick-and-place pipeline showing perception through actuation.",
    src: "/sys_arch/SystemPipeline.png",
  },
  {
    title: "Pick Sequence Flow",
    description: "Step-by-step breakdown of the pick workflow from scanning to retract.",
    src: "/sys_arch/PickSequenceFlow.png",
  },
  {
    title: "Drop-off Routing Logic",
    description: "Logic used to route items to the appropriate drop-off locations by AR tag.",
    src: "/sys_arch/Drop-offRoutingLogic.png",
  },
  {
    title: "Data Streams",
    description: "ROS 2 node connectivity covering camera input, detection, and robot control.",
    src: "/sys_arch/DataStreams.png",
  },
  {
    title: "Topic & Service Summary",
    description: "Topic and service wiring across the ROS 2 stack for GroceryGizmo.",
    src: "/sys_arch/TopicAndServiceSummary.png",
  },
  {
    title: "TF Frame Hierarchy",
    description: "Pose graph showing the kinematic chain and AR marker frames.",
    src: "/sys_arch/TFFrameHierarchy.png",
  },
  {
    title: "Hardware Components",
    description: "Hardware layout connecting control PC, sensors, and the TM5-700 arm.",
    src: "/sys_arch/HardwareComponents.png",
  },
];

const teamContributions = [
  {
    name: "Arya Sasikumar",
    contributions: [
      "Integrated the TM5-700 workcell with fridge fixtures and tuned approach trajectories for cramped shelving.",
      "Modeled and fabricated the eye-in-hand RealSense mount and removable AR tag anchors for grocery items.",
    ],
  },
  {
    name: "Gursimar Virk",
    contributions: [
      "Led manipulation testing, including grasp offset calibration and fridge entry sequencing.",
      "Developed safety playbooks covering motion stop conditions, soft limits, and manual recovery procedures.",
    ],
  },
  {
    name: "Yamuna Rao",
    contributions: [
      "Built the perception stack for AR tag detection, TF broadcasting, and nearest-object selection logic.",
      "Benchmarked IK latency and optimized MoveIt2 request batching for quicker pick-place cycles.",
    ],
  },
  {
    name: "Divya Krishnaswamy",
    contributions: [
      "Authored the ROS 2 task manager that sequences sensing, planning, and actuation for each grocery item.",
      "Managed gripper force profiling and validation on fragile items to prevent bruising or slips.",
    ],
  },
  {
    name: "Patrick O’Connor",
    contributions: [
      "Implemented the Next.js project site, system dashboards, and logging utilities for demo prep.",
      "Maintained integration scripts between the TM driver, Modbus gripper control, and dashboard telemetry.",
    ],
  },
];

const hardwareComponents = [
  {
    name: "Omron TM5-700 Collaborative Arm",
    detail:
      "Six degree-of-freedom manipulator executing pick-and-place trajectories with 700 mm reach and built-in safety monitoring.",
  },
  {
    name: "Robotiq 2F-85 Gripper",
    detail:
      "Modbus-controlled parallel gripper with configurable closure values matched to each AR tag class.",
  },
  {
    name: "Intel RealSense D435",
    detail:
      "Eye-in-hand RGB-D camera providing aligned depth for ArUco pose estimation in cluttered fridge scenes.",
  },
  {
    name: "Control PC (Ubuntu 22.04)",
    detail:
      "Runs ROS 2 Humble within Distrobox, MoveIt2 planners, Tkinter GUI, and trajectory logging utilities.",
  },
  {
    name: "Custom AR Tag Fixtures",
    detail:
      "3D-printed mounts and laminated tags that survive condensation while giving the camera crisp fiducials.",
  },
];

const softwareModules = [
  {
    name: "Perception Pipeline",
    bullets: [
      "Intel RealSense ROS 2 driver publishes synchronized RGB and depth frames to /camera topics.",
      "Custom ArUco detector service thresholds markers and broadcasts marker TF frames into tf2.",
      "Camera-to-base calibration maintains accurate transforms for downstream planning queries.",
    ],
  },
  {
    name: "Task Manager",
    bullets: [
      "robot_gui_recorder node maintains a priority queue of detected groceries by reach distance.",
      "Requests MoveIt2 /compute_ik for approach, grasp, and place poses with collision margins.",
      "Streams gripper commands and logs telemetry for replay during debugging sessions.",
    ],
  },
  {
    name: "Motion Execution",
    bullets: [
      "tm_driver wrapper sends joint-space trajectories with safety velocity caps and soft limits.",
      "Interlocks fridge entry waypoints to keep the elbow clear of door edges and shelving.",
      "Exception handling pauses the cycle on TF dropout, gripper timeouts, or unexpected torque spikes.",
    ],
  },
];

const systemWorkflow = [
  "RealSense D435 publishes aligned RGB-D frames through realsense2_camera_node.",
  "ar_tag_detector_service identifies grocery and shelf markers, pushing transforms into tf2.",
  "robot_gui_recorder selects the nearest viable grocery tag and pairs it with its fridge destination.",
  "MoveIt2 /compute_ik solves approach, grasp, and placement joint targets with collision checks.",
  "tm_driver executes the point-to-point trajectory while logging joint states for verification.",
  "Modbus gripper closes on the item, confirms force, and reopens after the placement target is reached.",
  "System resets to the scan pose, updates remaining inventory, and loops until the queue is empty.",
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
      "8 out of 10 grocery items were placed correctly during final integrated dry runs with door clearance checks.",
  },
  {
    title: "Average cycle time",
    detail:
      "72 seconds from detection to placement, dominated by cautious fridge entry and exit waypoints.",
  },
  {
    title: "System reliability",
    detail:
      "No collisions recorded after adding approach offsets; two runs paused for temporary TF dropouts that recovered automatically.",
  },
];

const resultMedia = [
  { src: "/arm2.jpg", title: "Arm reaching into fridge" },
  { src: "/ARCo_detection.JPEG", title: "AR tag pose detection" },
  { src: "/ros.JPG", title: "ROS graph overview" },
];

const demoVideoUrl = "https://drive.google.com/file/d/1GroceryGizmoDemo/view?usp=sharing";

const additionalMaterials = [
  {
    label: "Codebase",
    description: "Full Next.js site and project assets stored in this repository (FinalProject/grocerygizmo).",
    href: "https://github.com/pchrisoc/GroceryGizmo/tree/main/FinalProject/grocerygizmo",
  },
  {
    label: "ROS 2 launch + nodes",
    description: "Perception, task orchestration, and TM driver wrappers documented in FinalProject/sysarch.js.",
    href: "https://github.com/pchrisoc/GroceryGizmo/blob/main/FinalProject/sysarch.js",
  },
  {
    label: "CAD models",
    description: "3MF assets for camera mounts and workspace props (FinalProject/ARCO.3mf, Cube1.3mf, Sphere.3mf, etc.).",
    href: "https://github.com/pchrisoc/GroceryGizmo/tree/main/FinalProject",
  },
  {
    label: "Datasheets",
    description: "Intel RealSense D435, Omron TM5-700, and Robotiq 2F-85 references for hardware configuration.",
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

  const scrollTo = (id: string) => {
    const el = document.getElementById(id);
    if (el) el.scrollIntoView({ behavior: "smooth" });
  };

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
            <Button color="inherit" onClick={() => scrollTo("introduction")} sx={navButtonSx}>
              Introduction
            </Button>
            <Button color="inherit" onClick={() => scrollTo("team")} sx={navButtonSx}>
              Team
            </Button>
            <Button color="inherit" onClick={() => scrollTo("design")} sx={navButtonSx}>
              Design
            </Button>
            <Button color="inherit" onClick={() => scrollTo("implementation")} sx={navButtonSx}>
              Implementation
            </Button>
            <Button color="inherit" onClick={() => scrollTo("viewer")} sx={navButtonSx}>
              3D Models
            </Button>
            <Button color="inherit" onClick={() => scrollTo("results")} sx={navButtonSx}>
              Results
            </Button>
            <Button color="inherit" onClick={() => scrollTo("conclusion")} sx={navButtonSx}>
              Conclusion
            </Button>
          </Box>

          <IconButton
            sx={{
              display: { xs: "flex", md: "none" },
              transition: "all 0.3s ease",
              border: collapsed ? "1px solid rgba(255,255,255,0.2)" : "none",
              borderRadius: 2,
              p: collapsed ? 0.5 : 1,
            }}
          >
            <MenuIcon />
          </IconButton>
        </Toolbar>
      </AppBar>

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
                A six-axis Omron robot arm that identifies groceries using AR
                tags and autonomously loads them into a real refrigerator.
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
                variant="pulse"
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
                    Omron TM5-700 + Robotiq Gripper
                  </Typography>
                  <Typography variant="body2" sx={{ color: "grey.400", lineHeight: 1.6 }}>
                    Drag, pinch, and orbit the full TM5 model to explore our workspace clearances and
                    wrist-mounted RealSense rig directly in the browser.
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
            variant="lift"
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
                For many people, putting groceries into the fridge is more than a chore — it’s a physical challenge. Reaching into deep shelves, bending down repeatedly, lifting items overhead, and navigating crowded compartments can be exhausting or unsafe for elderly adults, disabled individuals, short users, or anyone with limited mobility.
              </Typography>
              <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                And beyond the physical strain, there’s the mental load: figuring out where everything fits in a cluttered, irregular fridge layout. Robots don’t get frustrated by tight spaces or overstuffed shelves — but getting them to understand and navigate those spaces is surprisingly hard.
              </Typography>
            </CardContent>
          </ScrollReactiveCard>
          <Grid container spacing={3}>
            <Grid size={{ xs: 12, md: 6 }}>
              <ScrollReactiveCard
                accentKey="introduction-solution"
                variant="slide"
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
                    GroceryGizmo is an autonomous fridge-loading robot that uses AR tags, computer vision, and a 6-DOF Omron arm to handle both the seeing and the doing:
                  </Typography>
                  <Box component="ul" sx={{ pl: 3, color: "grey.200", lineHeight: 1.7, m: 0 }}>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Sensing: The camera detects AR-tagged grocery items and AR-tagged fridge placements.
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Planning: The system chooses the closest item, works out where it belongs, and computes a safe pose in the robot’s base frame.
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        Actuation: The arm picks up the item and places it exactly where it’s meant to go — even in tight, awkward fridge compartments.
                      </Typography>
                    </li>
                  </Box>
                  <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                    It’s a complete end-to-end pipeline designed to make a robot genuinely useful in a real household task.
                  </Typography>
                </CardContent>
              </ScrollReactiveCard>
            </Grid>
            <Grid size={{ xs: 12, md: 6 }}>
              <ScrollReactiveCard
                accentKey="introduction-impact"
                variant="tilt"
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
                    As AI reshapes the world, we want robotics to support people — not replace meaningful work, but relieve the exhausting, repetitive, or physically inaccessible parts of daily life.
                  </Typography>
                  <Box component="ul" sx={{ pl: 3, color: "grey.200", lineHeight: 1.7, m: 0 }}>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        elderly individuals stay independent,
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        people with limited mobility avoid risky bending or lifting,
                      </Typography>
                    </li>
                    <li>
                      <Typography component="span" sx={{ color: "grey.200" }}>
                        and busy households offload a chore that always seems to take too long.
                      </Typography>
                    </li>
                  </Box>
                  <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                    GroceryGizmo is a step toward home robots that make everyday living a little easier — one grocery item at a time.
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
            A multidisciplinary team from EECS, ME, BioE & Business.
          </Typography>

          <Grid container spacing={3}>
            {teamMembers.map((m, index) => (
              <Grid size={{ xs: 12, md: 6 }} key={m.name}>
                <ScrollReactiveCard
                  accentKey={`team-${m.name}`}
                  variant={REACTIVE_VARIANTS[index % REACTIVE_VARIANTS.length]}
                  sx={{
                    borderRadius: 3,
                    border: "1px solid rgba(255,255,255,0.08)",
                  }}
                >
                  <CardContent
                    sx={{
                      display: "flex",
                      alignItems: "stretch",
                      p: 0,
                      height: 200,
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
                      <Typography variant="body2" sx={{ color: "grey.400" }}>
                        {m.background}
                      </Typography>
                    </Box>
                  </CardContent>
                </ScrollReactiveCard>
              </Grid>
            ))}
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
            variant="slide"
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
                    The robot must reliably see AR-tagged objects on the counter and compute their 3D position in space, regardless of object shape or orientation.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Identify placement locations inside the fridge
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    AR tags inside the refrigerator provide fixed anchor points the robot can recognize and navigate toward.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Determine which item to pick first
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    The system should automatically choose the closest grocery item to minimize unnecessary motion and make the workflow efficient.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Transform camera detections into robot coordinates
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    Accurate frame transformations are essential so the Omron arm can move to the correct pre-grasp and placement poses.
                  </Typography>
                </Box>
                <Box>
                  <Typography variant="subtitle2" sx={{ fontWeight: 600, color: "grey.100" }}>
                    Grasp and place items safely
                  </Typography>
                  <Typography sx={{ color: "grey.300", lineHeight: 1.7 }}>
                    The robot must approach items with a safe offset, lower the gripper for a stable grasp, enter the fridge without collisions, and set items into their tagged locations.
                  </Typography>
                </Box>
              </Box>
            </CardContent>
          </ScrollReactiveCard>

          <ScrollReactiveCard
            accentKey="design-functionality"
            variant="tilt"
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
                    Perceive items on a counter
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Decide where each item belongs
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Plan reachable poses for grasping and placing
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Move the arm through the constrained fridge space
                  </Typography>
                </li>
                <li>
                  <Typography component="span" sx={{ color: "grey.200" }}>
                    Gently and accurately place groceries in their correct spots
                  </Typography>
                </li>
              </Box>
              <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                Together, these criteria ensure the robot behaves in a predictable, useful, and safe way — bringing us closer to assistive home robotics that can handle everyday tasks without constant human oversight.
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
            The GroceryGizmo stack combines collaborative-grade hardware with ROS 2 software, MoveIt2 planning, and a lightweight GUI to shepherd each grocery item from the counter into its tagged shelf. Below are the key components, supporting software, and the full pipeline we deployed.
          </Typography>
          <Typography variant="h6" sx={{ fontWeight: 700, mb: 2 }}>
            System Architecture
          </Typography>
          <Typography sx={{ color: "grey.400", mb: 4 }}>
            Expand each diagram to explore the perception, planning, and hardware
            layers of GroceryGizmo.
          </Typography>

          <Grid container spacing={3}>
            {systemArchitectureImages.map((item, index) => (
              <Grid key={item.title} size={{ xs: 12, md: 6, lg: 4 }}>
                <ScrollReactiveCard
                  accentKey={`system-architecture-${item.title}`}
                  variant={REACTIVE_VARIANTS[index % REACTIVE_VARIANTS.length]}
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
      {/* 3D MODELS GALLERY */}
      {/* =========================================== */}

      <Box id="viewer" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            3D Models Gallery
          </Typography>
          <Typography sx={{ color: "grey.400", mb: 4 }}>
            Browse the simplified CAD assets we use for workspace layout,
            storage design, and reach studies inside the planning pipeline.
          </Typography>

          <Grid container spacing={4}>
            {modelGallery.map((model, index) => (
              <Grid size={{ xs: 12, md: 4 }} key={model.title}>
                <ScrollReactiveCard
                  accentKey={`model-gallery-${model.title}`}
                  variant={REACTIVE_VARIANTS[index % REACTIVE_VARIANTS.length]}
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
            We validated GroceryGizmo on a countertop-to-fridge workflow with AR-tagged groceries of varying shapes. The system performed full autonomous pick-and-place cycles, logged motion telemetry, and showcased the interaction between perception, planning, and manipulation.
          </Typography>
          <ScrollReactiveCard
            accentKey="results-demo"
            variant="ripple"
            sx={{
              borderRadius: 3,
              border: "1px solid rgba(255,255,255,0.08)",
            }}
          >
            <CardContent sx={{ display: "flex", flexDirection: "column", gap: 1 }}>
              <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                Demo Video
              </Typography>
              <Typography sx={{ color: "grey.300" }}>
                Watch the full pick-and-place routine, including detection overlays and fridge placement, in our recorded demo.
              </Typography>
              <MuiLink
                href={demoVideoUrl}
                target="_blank"
                rel="noopener noreferrer"
                underline="hover"
                color="primary.main"
              >
                GroceryGizmo Demo Video
              </MuiLink>
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
            GroceryGizmo met its core design criteria by detecting groceries, aligning fridge placements, and executing constrained pick-and-place trajectories. The robot handled multiple item classes without collisions and maintained consistent localization accuracy across runs.
          </Typography>

          <Typography variant="h6" sx={{ fontWeight: 600, mb: 1.5 }}>
            Challenges
          </Typography>
          <Box component="ul" sx={{ pl: 3, color: "grey.300", lineHeight: 1.7, mb: 3 }}>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Managing depth sensor noise on reflective packaging required tuning detection thresholds and adding pass-through filters.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Coordinating fridge entry angles with a wrist-mounted camera demanded iterative offset calibration.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Ensuring Modbus gripper timing aligned with TM driver feedback loops introduced additional state checks.
              </Typography>
            </li>
          </Box>

          <Typography variant="h6" sx={{ fontWeight: 600, mb: 1.5 }}>
            Known Limitations & Future Work
          </Typography>
          <Box component="ul" sx={{ pl: 3, color: "grey.300", lineHeight: 1.7 }}>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Fridge organization still depends on AR tags; expanding to vision-based item classification is a natural extension.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                The current workflow assumes the fridge door is already open; automating door manipulation remains future work.
              </Typography>
            </li>
            <li>
              <Typography component="span" sx={{ color: "grey.300" }}>
                Adding tactile sensing could further reduce slip risk for heavier or deformable items.
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
