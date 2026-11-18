// Fusion reports left-positive body bearings. Guidance uses right-positive bearings.
// Keep the original receiver measurements available to logging, admission and safety.
export function withFollowBearing(telemetry, state, now, uwbTimeoutMs) {
    if (!state) return telemetry;
    const uwb = telemetry.uwb;
    const result = state.result;
    const validRange = state.uwb.valid && uwb.online && uwb.fresh && uwb.measurementStatus === 'trusted'
        && Number.isFinite(uwb.measurementUpdatedAt) && now >= uwb.measurementUpdatedAt
        && now - uwb.measurementUpdatedAt <= uwbTimeoutMs && Number.isFinite(uwb.distanceM) && uwb.distanceM > 0;
    if (!validRange) return { ...telemetry, guidanceSource: 'unavailable', fusionReason: 'uwb_unavailable',
        uwb: { ...uwb, fresh: false } };
    const usable = state.enabled && state.status === 'fused'
        && result?.source === 'uwb+vision' && Number.isFinite(result.bearingDeg)
        && result.measurementUpdatedAt === uwb.measurementUpdatedAt;
    const guidanceSource = usable ? 'uwb+vision' : 'uwb';
    return { ...telemetry, guidanceSource, fusionReason: state.reason,
        uwb: usable ? { ...uwb, followBearingDeg: -result.bearingDeg } : uwb };
}
