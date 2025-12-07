"use client";

import React from "react";
import {
  AppBar,
  Box,
  Button,
  Card,
  CardContent,
  Container,
  CssBaseline,
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
} from "@mui/material";
import Grid from "@mui/material/Grid"; // IMPORTANT: Classic Grid import
import MenuIcon from "@mui/icons-material/Menu";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import Image from "next/image";

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
const teamMembers = [
  {
    name: "Arya Sasikumar",
    background:
      "Mechanical Engineering & Business major interested in robotics. Well-versed in Solidworks, ROS, Python, and C++.",
  },
  {
    name: "Gursimar Virk",
    background:
      "Mechanical Engineering major and Material Science minor interested in robotics. Works with humanoids; skilled in Solidworks, ROS, Python, and C++.",
  },
  {
    name: "Yamuna Rao",
    background:
      "EECS major interested in robotics/optimization. Experience with CV, Python, C, PyTorch, and TensorFlow.",
  },
  {
    name: "Divya Krishnaswamy",
    background:
      "Bioengineering major & EECS minor interested in surgical robotics. Experience with Solidworks, C++, Python, and embedded systems.",
  },
  {
    name: "Patrick O’Connor",
    background:
      "Third-year EECS major with interest in EE & robotics. Experience with Fusion, KiCad, and Python.",
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
  const scrollTo = (id: string) => {
    const el = document.getElementById(id);
    if (el) el.scrollIntoView({ behavior: "smooth" });
  };

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />

      {/* NAVBAR */}
      <AppBar position="sticky" elevation={0} color="transparent">
        <Toolbar>
          <Typography
            variant="h6"
            sx={{ flexGrow: 1, fontWeight: 600, cursor: "pointer" }}
            onClick={() => scrollTo("hero")}
          >
            GroceryGizmo
          </Typography>

          <Box sx={{ display: { xs: "none", md: "flex" }, gap: 2 }}>
            <Button color="inherit" onClick={() => scrollTo("team")}>Team</Button>
            <Button color="inherit" onClick={() => scrollTo("abstract")}>Abstract</Button>
            <Button color="inherit" onClick={() => scrollTo("project")}>Project</Button>
            <Button color="inherit" onClick={() => scrollTo("tasks")}>Tasks</Button>
            <Button color="inherit" onClick={() => scrollTo("bom")}>BOM</Button>
            <Button color="inherit" onClick={() => scrollTo("gallery")}>Gallery</Button>
            <Button color="inherit" onClick={() => scrollTo("code")}>Code</Button>
          </Box>

          <IconButton sx={{ display: { xs: "flex", md: "none" } }}>
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
                GroceryGizmo:
                <br />
                <Box component="span" sx={{ color: "primary.main" }}>
                  Robot-Assisted Grocery Loading
                </Box>
              </Typography>

              <Typography variant="body1" sx={{ mb: 3, color: "grey.300" }}>
                A six-axis Omron robot arm that identifies groceries using AR
                tags and autonomously loads them into a real refrigerator.
              </Typography>

              <Box sx={{ display: "flex", gap: 2, flexWrap: "wrap" }}>
                <Button variant="contained" size="large" onClick={() => scrollTo("project")}>
                  View Project Details
                </Button>
                <Button variant="outlined" size="large" onClick={() => scrollTo("gallery")}>
                  View Gallery
                </Button>
              </Box>
            </Grid>

            {/* RIGHT SIDE */}
            <Grid size={{ xs: 12, md: 5 }}>
              <Card
                sx={{
                  borderRadius: 3,
                  overflow: "hidden",
                  backdropFilter: "blur(12px)",
                  border: "1px solid rgba(255,255,255,0.06)",
                }}
              >
                <Box sx={{ position: "relative", height: 260 }}>
                  <Image
                    src="/arm.jpg"
                    alt="Omron TM5-700 Arm staging groceries"
                    fill
                    style={{ objectFit: "cover" }}
                  />
                </Box>
                <CardContent>
                  <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                    Omron TM5-700 + Robotiq Gripper
                  </Typography>
                  <Typography variant="body2" sx={{ color: "grey.400", mt: 1 }}>
                    Industrial hardware deployed in a real kitchen to explore
                    perception, manipulation, and task-level autonomy.
                  </Typography>
                </CardContent>
              </Card>
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
            {teamMembers.map((m) => (
              <Grid size={{ xs: 12, sm: 6, md: 4 }} key={m.name}>
                <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                  <CardContent>
                    <Typography variant="h6" sx={{ fontWeight: 600, mb: 1.5 }}>
                      {m.name}
                    </Typography>
                    <Typography variant="body2" sx={{ color: "grey.400" }}>
                      {m.background}
                    </Typography>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* ABSTRACT */}
      {/* =========================================== */}

      <Box id="abstract" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            Abstract
          </Typography>

          <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
            <CardContent>
              <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                We will use an industrial six-axis robot arm acquired through
                Omron Robotics to autonomously load groceries into a household
                refrigerator. The system includes a mobile base, vision-based
                item identification via AR tags, and a gripper capable of
                handling cartons, cans, and loose produce. After mapping the
                kitchen, the robot detects tagged groceries, plans collision-free
                trajectories, and places each item on a matching AR-tagged shelf
                inside the fridge. Safety is ensured through soft limits,
                supervised operation, and an emergency stop. The prototype
                demonstrates practical home automation and provides a platform
                for experimenting with perception, grasp planning, and task
                sequencing in cluttered environments—supporting elderly,
                disabled, or recovering individuals with day-to-day tasks.
              </Typography>
            </CardContent>
          </Card>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* PROJECT DESCRIPTION (Goals, Architecture, etc.) */}
      {/* =========================================== */}

      <Box id="project" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 3 }}>
            Project Description
          </Typography>

          {/* GOALS + ARCHITECTURE */}
          <Grid container spacing={4}>
            {/* Goals */}
            <Grid size={{ xs: 12, md: 6 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1.5 }}>
                    Project Goals
                  </Typography>

                  <Typography component="ul" sx={{ pl: 2, mb: 2 }}>
                    <li>Identify grocery items using AR tags</li>
                    <li>Pick and load groceries into a refrigerator</li>
                    <li>Match fridge shelf AR tags to object AR tags</li>
                  </Typography>

                  <Typography variant="subtitle2" sx={{ mb: 1 }}>
                    Stretch Goals
                  </Typography>

                  <Typography component="ul" sx={{ pl: 2 }}>
                    <li>Automatically opening the fridge door</li>
                    <li>Using pretrained CV models instead of AR tags</li>
                    <li>Placing items by semantic labels (e.g., “Dairy”)</li>
                    <li>Sorting groceries by expiration date</li>
                  </Typography>
                </CardContent>
              </Card>
            </Grid>

            {/* Architecture */}
            <Grid size={{ xs: 12, md: 6 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1.5 }}>
                    Software & Hardware Architecture
                  </Typography>
                  <Typography sx={{ color: "grey.200", lineHeight: 1.7 }}>
                    A ROS 2 system using an Omron 6-axis arm, Robotiq gripper,
                    and RGB-D camera. AR tags provide identity and location of
                    groceries. A task manager controls the robot state. MoveIt +
                    inverse kinematics generate collision-free grasp and place
                    trajectories. Force sensing prevents slipping or crushing.
                  </Typography>
                </CardContent>
              </Card>
            </Grid>
          </Grid>

          {/* Sensing / Planning / Actuation */}
          <Grid container spacing={4} sx={{ mt: 4 }}>
            <Grid size={{ xs: 12, md: 4 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1 }}>Sensing</Typography>
                  <Typography sx={{ color: "grey.200" }}>
                    RGB-D camera detects AR tags on groceries & fridge shelves;
                    gripper force sensor ensures safe grasps.
                  </Typography>
                </CardContent>
              </Card>
            </Grid>

            <Grid size={{ xs: 12, md: 4 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1 }}>Planning</Typography>
                  <Typography sx={{ color: "grey.200" }}>
                    MoveIt + IK compute collision-free pick-and-place
                    trajectories around obstacles like fridge doors.
                  </Typography>
                </CardContent>
              </Card>
            </Grid>

            <Grid size={{ xs: 12, md: 4 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1 }}>Actuation</Typography>
                  <Typography sx={{ color: "grey.200" }}>
                    Omron TM5-700 arm and Robotiq gripper manipulate groceries
                    safely using category-based grip forces.
                  </Typography>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* TASKS */}
      {/* =========================================== */}

      <Box id="tasks" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 3 }}>
            Tasks
          </Typography>

          <Grid container spacing={4}>
            {/* Build */}
            <Grid size={{ xs: 12, md: 6 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1.5 }}>Build</Typography>
                  <Typography component="ul" sx={{ pl: 2 }}>
                    <li>Manufacture AR tag stickers</li>
                    <li>Apply tags to groceries and fridge shelf locations</li>
                  </Typography>
                </CardContent>
              </Card>
            </Grid>

            {/* Code */}
            <Grid size={{ xs: 12, md: 6 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1.5 }}>Code</Typography>
                  <Typography component="ul" sx={{ pl: 2 }}>
                    <li>Camera Node → /camera/image_raw</li>
                    <li>AR Node → /aruco/poses, /tf</li>
                    <li>Pose Estimator → PoseStamped in camera frame</li>
                    <li>Object Selector → /target_pose (base_link)</li>
                  </Typography>
                </CardContent>
              </Card>
            </Grid>
          </Grid>

          {/* Test */}
          <Grid container spacing={4} sx={{ mt: 4 }}>
            <Grid size={{ xs: 12, md: 6 }}>
              <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
                <CardContent>
                  <Typography variant="h6" sx={{ mb: 1.5 }}>Test</Typography>
                  <Typography component="ul" sx={{ pl: 2 }}>
                    <li>Begin with fridge open and item on counter</li>
                    <li>Perform pick + move sequence</li>
                    <li>Place item in correct AR-tagged fridge location</li>
                  </Typography>
                </CardContent>
              </Card>
            </Grid>
          </Grid>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* BILL OF MATERIALS */}
      {/* =========================================== */}

      <Box id="bom" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 3 }}>
            Bill of Materials
          </Typography>

          <Typography variant="h6">5.1 Lab Resources</Typography>
          <Typography sx={{ color: "grey.400", mb: 3 }}>N/A</Typography>

          <Typography variant="h6">5.2 Other Robotic Platforms</Typography>
          <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)", mb: 3 }}>
            <Table size="small">
              <TableHead>
                <TableRow>
                  <TableCell>Item</TableCell>
                  <TableCell>Qty</TableCell>
                  <TableCell>Owner / Location</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                <TableRow>
                  <TableCell>Omron TM5-700</TableCell>
                  <TableCell>1</TableCell>
                  <TableCell>Arya / Living Room</TableCell>
                </TableRow>
              </TableBody>
            </Table>
          </Card>

          <Typography variant="h6">5.3 Other Purchases</Typography>
          <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.08)" }}>
            <Table size="small">
              <TableHead>
                <TableRow>
                  <TableCell>Item</TableCell>
                  <TableCell>Qty</TableCell>
                  <TableCell>Justification</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                <TableRow>
                  <TableCell>Grocery Items</TableCell>
                  <TableCell>10+</TableCell>
                  <TableCell>Objects to load into fridge</TableCell>
                </TableRow>
                <TableRow>
                  <TableCell>AR Tags</TableCell>
                  <TableCell>15+</TableCell>
                  <TableCell>Identify groceries & shelves</TableCell>
                </TableRow>
                <TableRow>
                  <TableCell>Sticker Paper</TableCell>
                  <TableCell>15+</TableCell>
                  <TableCell>Attach AR tags to produce</TableCell>
                </TableRow>
              </TableBody>
            </Table>
          </Card>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* GALLERY */}
      {/* =========================================== */}

      <Box id="gallery" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            Robot Gallery
          </Typography>
          <Typography sx={{ color: "grey.400", mb: 3 }}>
            Replace images in <code>/public/images/</code> with your own.
          </Typography>

          <ImageList variant="masonry" cols={3} gap={12}>
            {galleryItems.map((img) => (
              <ImageListItem key={img.src}>
                <Box
                  sx={{
                    position: "relative",
                    borderRadius: 3,
                    overflow: "hidden",
                    border: "1px solid rgba(255,255,255,0.06)",
                  }}
                >
                  <Box sx={{ position: "relative", height: 220 }}>
                    <Image
                      src={img.src}
                      alt={img.title}
                      fill
                      style={{ objectFit: "cover" }}
                    />
                  </Box>
                  <Box sx={{ p: 1.5 }}>
                    <Typography variant="subtitle2">{img.title}</Typography>
                  </Box>
                </Box>
              </ImageListItem>
            ))}
          </ImageList>
        </Container>
      </Box>

      <Divider />

      {/* =========================================== */}
      {/* CODE BLOCK */}
      {/* =========================================== */}

      <Box id="code" sx={{ py: 8 }}>
        <Container maxWidth="lg">
          <Typography variant="h4" sx={{ fontWeight: 700, mb: 2 }}>
            ROS 2 Code Pipeline
          </Typography>
          <Typography sx={{ color: "grey.400", mb: 3 }}>
            Example pipeline showing vision → pose estimation → IK planning.
          </Typography>

          <Card sx={{ borderRadius: 3, border: "1px solid rgba(255,255,255,0.12)" }}>
            <CardContent>
              <Box
                component="pre"
                sx={{
                  m: 0,
                  p: 2,
                  fontFamily: "monospace",
                  fontSize: 13,
                  overflowX: "auto",
                  whiteSpace: "pre",
                }}
              >
                <code>{rosCodeSnippet}</code>
              </Box>
            </CardContent>
          </Card>
        </Container>
      </Box>

      <Divider />

      {/* FOOTER */}
      <Box sx={{ py: 4 }}>
        <Container maxWidth="lg">
          <Typography variant="body2" sx={{ color: "grey.500", textAlign: "center" }}>
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
    </ThemeProvider>
  );
}
