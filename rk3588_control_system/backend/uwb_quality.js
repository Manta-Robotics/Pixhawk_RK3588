// Shared veto for observations isolated by the bridge continuity guard.
// Legacy packets without diagnostics retain their existing admission rules.
export function uwbObservationQuarantined(sensor) {
  return ['suspect', 'quarantined', 'recovering'].includes(sensor?.occlusionDiagnostics?.state);
}
