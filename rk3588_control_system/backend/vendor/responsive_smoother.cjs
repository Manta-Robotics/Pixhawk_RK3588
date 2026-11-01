/**
 * UWB “响应优先”平滑算法；无串口、页面或设备依赖。
 *
 * 每个基站/信标组合使用一个独立实例。输入距离单位 cm、水平角单位度。
 * 按 40 Hz（每帧 25 ms）计算；可传入接收时间戳用于检测超过 500 ms 的间隔。
 * 心跳或断线时调用 reset()。ready=false 时不要使用平滑结果。
 *
 * 这个文件导出当前实时页面“响应优先”的数值逻辑，不判断信号是否视距或方位是否正确。
 */
class UwbResponsiveSmoother {
  static parameters = Object.freeze({
    warmupSamples: 5,
    periodMs: 25,
    resetGapMs: 500,
    estimateTauMs: 50,
    outputTauMs: 50,
    deadbandCm: 10,
    maxSpeedCmPerSecond: 350,
    maxAccelCmPerSecond2: 5000
  });

  constructor(parameters = {}) {
    this.parameters = Object.freeze({ ...UwbResponsiveSmoother.parameters, ...parameters });
    this.reset();
  }

  reset() {
    this.warmX = [];
    this.warmY = [];
    this.estimateX = null;
    this.estimateY = null;
    this.outputX = null;
    this.outputY = null;
    this.velocityX = 0;
    this.velocityY = 0;
    this.lastTimestampMs = null;
  }

  static median(values) {
    const ordered = [...values].sort((a, b) => a - b);
    const middle = ordered.length >> 1;
    return ordered.length % 2
      ? ordered[middle]
      : (ordered[middle - 1] + ordered[middle]) / 2;
  }

  static limitVector(x, y, limit) {
    const magnitude = Math.hypot(x, y);
    return magnitude > limit && magnitude > 0
      ? [x * limit / magnitude, y * limit / magnitude]
      : [x, y];
  }

  /**
   * @param {number} distanceCm 正值，厘米
   * @param {number} azimuthDeg 水平角，度；可为负或超过 360
   * @param {number|null} timestampMs 可选的同一单调时钟时间戳，毫秒
   * @returns {{ready:boolean, smoothedDistanceCm?:number,
   *   smoothedAzimuthDeg?:number, smoothedXcm?:number, smoothedYcm?:number,
   *   estimateXcm?:number, estimateYcm?:number, reason?:string}}
   */
  update(distanceCm, azimuthDeg, timestampMs = null) {
    const r = Number(distanceCm);
    const angle = Number(azimuthDeg);
    if (!Number.isFinite(r) || r <= 0 || !Number.isFinite(angle)) {
      this.reset();
      return { ready: false, reason: 'invalid_measurement' };
    }

    if (Number.isFinite(timestampMs)) {
      if (this.lastTimestampMs !== null &&
          timestampMs - this.lastTimestampMs > this.parameters.resetGapMs) {
        this.reset();
      }
      this.lastTimestampMs = timestampMs;
    }

    const p = this.parameters;
    const radians = angle * Math.PI / 180;
    const x = r * Math.cos(radians);
    const y = r * Math.sin(radians);

    if (this.estimateX === null) {
      this.warmX.push(x);
      this.warmY.push(y);
      if (this.warmX.length < p.warmupSamples) {
        return { ready: false, reason: 'warmup' };
      }
      this.estimateX = UwbResponsiveSmoother.median(this.warmX);
      this.estimateY = UwbResponsiveSmoother.median(this.warmY);
      this.outputX = this.estimateX;
      this.outputY = this.estimateY;
    } else {
      const dt = p.periodMs / 1000;
      const alpha = 1 - Math.exp(-p.periodMs / p.estimateTauMs);
      this.estimateX += alpha * (x - this.estimateX);
      this.estimateY += alpha * (y - this.estimateY);

      let dx = this.estimateX - this.outputX;
      let dy = this.estimateY - this.outputY;
      const distanceToEstimate = Math.hypot(dx, dy);
      if (distanceToEstimate <= p.deadbandCm) {
        dx = 0;
        dy = 0;
      } else {
        const factor = 1 - p.deadbandCm / distanceToEstimate;
        dx *= factor;
        dy *= factor;
      }

      const desired = UwbResponsiveSmoother.limitVector(
        dx * 1000 / p.outputTauMs,
        dy * 1000 / p.outputTauMs,
        p.maxSpeedCmPerSecond
      );
      const change = UwbResponsiveSmoother.limitVector(
        desired[0] - this.velocityX,
        desired[1] - this.velocityY,
        p.maxAccelCmPerSecond2 * dt
      );
      this.velocityX += change[0];
      this.velocityY += change[1];
      this.outputX += this.velocityX * dt;
      this.outputY += this.velocityY * dt;
    }

    const wrappedAngle = ((Math.atan2(this.outputY, this.outputX) * 180 / Math.PI) % 360 + 360) % 360;
    return {
      ready: true,
      smoothedDistanceCm: Math.hypot(this.outputX, this.outputY),
      smoothedAzimuthDeg: wrappedAngle,
      smoothedXcm: this.outputX,
      smoothedYcm: this.outputY,
      estimateXcm: this.estimateX,
      estimateYcm: this.estimateY
    };
  }
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = UwbResponsiveSmoother;
}
if (typeof globalThis !== 'undefined') {
  globalThis.UwbResponsiveSmoother = UwbResponsiveSmoother;
}
