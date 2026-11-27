// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0

// The scene of a plan episode: its objects and targets where the episode places them, in metres
// and degrees in the robot base frame (the frame of the simulated world and of MuJoCo).
export default function sceneItems(plan, episode) {
  if (!plan || !episode) return [];
  return (episode.placements || [])
    .map((p) => {
      const item = (plan.task?.items || []).find((i) => i.id === p.item_id);
      if (!item) return null;
      return {
        id: item.id,
        name: item.name,
        role: item.role === 'target' ? 'target' : 'object',
        x: p.x_mm / 1000,
        y: p.y_mm / 1000,
        yaw: p.yaw_deg || 0,
        size: (item.size_mm || (item.role === 'target' ? 80 : 30)) / 1000,
        color: item.color,
      };
    })
    .filter(Boolean);
}
