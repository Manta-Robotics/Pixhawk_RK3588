// The camera calibration and control gains use calibration-image pixels.
// Tracker images may be resized independently of the displayed video.
export function trackerControlPixels(dx, dy, vx, vy, metadata = {}, width = 1920, height = 1080) {
  if (![dx, dy, vx, vy].every(value => typeof value === 'number' && Number.isFinite(value))) {
    throw new Error('Invalid tracker coordinates or velocity');
  }
  const frameWidth = metadata.frame_w ?? metadata.frameW ?? width;
  const frameHeight = metadata.frame_h ?? metadata.frameH ?? height;
  if (![frameWidth, frameHeight, width, height].every(value => typeof value === 'number' && Number.isFinite(value) && value > 0)) {
    throw new Error('Invalid tracker image dimensions');
  }
  return { dx: dx * width / frameWidth, dy: dy * height / frameHeight,
    vx: vx * width / frameWidth, vy: vy * height / frameHeight };
}
