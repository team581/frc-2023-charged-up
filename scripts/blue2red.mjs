#!/usr/bin/env node

// @ts-check

import fs from "node:fs/promises";
import nodePath from "node:path";

const __dirname = nodePath.dirname(new URL(import.meta.url).pathname);

function serialize(path) {
  return JSON.stringify(path, null, 2);
}

function translate(waypoint) {
  const { x } = waypoint.anchorPoint;

  // The origin is on the left side
  // There is a center line at x = 8.27
  // We want to reflect the point across the center line

  waypoint.anchorPoint.x = 8.27 - (x - 8.27);
  waypoint.holonomicAngle = -waypoint.holonomicAngle + 180;
  waypoint.holonomicAngle %= 360;

  if (waypoint.prevControl) {
    waypoint.prevControl.x = 8.27 - (waypoint.prevControl.x - 8.27);
  }

  if (waypoint.nextControl) {
    waypoint.nextControl.x = 8.27 - (waypoint.nextControl.x - 8.27);
  }
}

const pathDir = nodePath.join(
  __dirname,
  "..",
  "src",
  "main",
  "deploy",
  "pathplanner"
);
const [, , ...rawFilenames] = process.argv;

let filenames = rawFilenames.map((rawFilename) =>
  rawFilename.endsWith(".path") ? rawFilename : `${rawFilename}.path`
);

// Default to all blue paths
if (filenames.length === 0) {
  const dir = await fs.readdir(pathDir);

  filenames = dir
    .filter(
      (filename) => filename.startsWith("Blue") && filename.endsWith(".path")
    )
    .map((absolutePath) => nodePath.basename(absolutePath));
}

for (const filename of filenames) {
  const pathBuf = await fs.readFile(nodePath.join(pathDir, filename));
  const path = JSON.parse(pathBuf.toString());

  for (const waypoint of path.waypoints) {
    translate(waypoint);
  }

  const outputPath = nodePath.join(
    pathDir,
    `Red${filename.slice("Blue".length)}`
  );
  await fs.writeFile(outputPath, serialize(path));
  console.log(outputPath);
}
