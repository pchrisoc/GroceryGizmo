# GroceryGizmo (EECS 106A Final Project)

Project site for **GroceryGizmo**: a robot grocery assistant that uses **AR tags**, **ROS 2**, and an **Omron TM5-700** arm to pick groceries from a counter and place them into a real refrigerator.

This repo contains the **Next.js App Router** site used for the final writeup, media gallery, interactive system diagrams, and an in-browser **OBJ/MTL 3D model viewer**.

### Highlights

- Single-page report layout with sections: Introduction, Team, Design, Implementation, Code, 3D Models, Results, Conclusion.
- 3D viewer built with `@react-three/fiber` + `three` (loads `.obj` + optional `.mtl` from `public/`).
- Responsive navigation with desktop + mobile drawer.
- Static assets (team photos, demo images, system architecture diagrams) served from `public/`.

## Tech Stack

- Next.js (App Router)
- React + TypeScript
- Material UI (MUI)
- three.js + react-three-fiber (`@react-three/fiber`, `@react-three/drei`)

## Local Development

### Prerequisites

- Node.js 18+ recommended
- `pnpm` recommended (a `pnpm-lock.yaml` is included)

### Install + run

```bash
pnpm install
pnpm dev
```

Then open `http://localhost:3000`.

### Other scripts

```bash
pnpm build
pnpm start
pnpm lint
```

## Project Layout

- `src/app/page.tsx` — Main one-page site content (sections, data, media lists, etc.)
- `src/app/components/ObjViewer.tsx` — Client-only OBJ/MTL viewer component (OrbitControls + optional autorotate)
- `public/models/` — 3D assets (`.obj` + `.mtl`)
- `public/sys_arch/` — System architecture diagrams used in the Implementation section
- `public/` — Photos and additional images used throughout the page

## Updating Content

### Add a new 3D model

1. Put the files in `public/models/` (e.g. `my_part.obj` and `my_part.mtl`).
2. Add an entry to the `modelGallery` array in `src/app/page.tsx`.

### Add/replace images

- Drop images into `public/` (or a subfolder) and reference them with absolute paths like `/arm.jpg` or `/sys_arch/SystemPipeline.png`.

## Notes

- The 3D viewer is imported with `dynamic(..., { ssr: false })` to avoid server-side rendering issues with WebGL/Three.js.
- If a model has no `.mtl`, the viewer falls back to a reasonable default material color.

## Demo Video

The Results section embeds the demo video via Google Drive:

- `https://drive.google.com/file/d/1sSn59fT9UaXw09uU73KojV-ZqYzFssF2/preview`

## Deployment

This is a standard Next.js app and can be deployed anywhere Next.js is supported (Vercel, a VM, or a robot-side console). A typical deploy flow is:

```bash
pnpm install --frozen-lockfile
pnpm build
pnpm start
```

## Credits

GroceryGizmo team: Arya Sasikumar, Gursimar Virk, Yamuna Rao, Divya Krishnaswamy, Patrick O’Connor.
