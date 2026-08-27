import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";

const testDir = path.dirname(fileURLToPath(import.meta.url));
const appRoot = path.resolve(testDir, "..");
const dashboardHtml = fs.readFileSync(path.join(appRoot, "frontend", "index.html"), "utf8");
const gimbalScript = fs.readFileSync(path.join(appRoot, "frontend", "js", "page_gimbal.js"), "utf8");
const mobileHtml = fs.readFileSync(path.join(appRoot, "frontend", "mobile-preview-kimi-k26.html"), "utf8");
const transportScript = fs.readFileSync(path.join(appRoot, "frontend", "js", "manta-app-transport.js"), "utf8");
const serverScript = fs.readFileSync(path.join(appRoot, "backend", "server.js"), "utf8");

test("dashboard exposes a direct UWB Beacon entry", () => {
    assert.match(
        dashboardHtml,
        /id="btnUwbBeacon"[^>]*href="gimbal\.html\?mode=beacon"|href="gimbal\.html\?mode=beacon"[^>]*id="btnUwbBeacon"/,
    );
    assert.match(dashboardHtml, />进入 UWB Beacon<\/a>/);
});

test("gimbal entry query automatically starts beacon tracking", () => {
    assert.match(gimbalScript, /new URLSearchParams\(window\.location\.search\)/);
    assert.match(gimbalScript, /requestedTrackModeFromLocation\(\) === "beacon"/);
    assert.match(gimbalScript, /startTrackMode\("beacon"\)/);
});

test("live mobile frontend exposes and transports beacon mode", () => {
    assert.match(mobileHtml, /data-gimbal-mode="beacon">UWB 信标<\/button>/);
    assert.match(mobileHtml, /payload\.trackMode==="beacon"/);
    assert.match(transportScript, /action === "beacon"/);
});

test("beacon toolbar exposes the nine-point calibration workflow", () => {
    assert.match(mobileHtml, /id="gimbalBeaconCalibration"/);
    assert.match(mobileHtml, /id="beaconCalibrationModal"/);
    assert.match(transportScript, /\/api\/gimbal\/beacon\/calibration\/capture/);
    assert.match(serverScript, /generateBeaconCalibrationPoints\(\)/);
    assert.match(serverScript, /fitBeaconCalibration\(session\.samples\)/);
    assert.match(serverScript, /let gimbalBeaconCalibrationState = \{[\s\S]*?total: 9/);
    assert.match(mobileHtml, /id="beaconCalibrationCanvas"/);
    assert.match(mobileHtml, /九点校准/);
    assert.doesNotMatch(mobileHtml, /八点校准/);
    assert.match(mobileHtml, /if\(state\.gimbalMode==="beacon"\)return/);
    assert.match(serverScript, /collectBeaconCalibrationReading\(\)/);
    assert.match(serverScript, /rawDistanceM/);
    assert.match(serverScript, /result\.model !== 'rigid_3d'/);
    assert.match(serverScript, /stabilizeBeaconUwb\(now,/);
});

test("boat auto-follow is controlled from the drive module, not the gimbal view", () => {
    assert.doesNotMatch(mobileHtml, /id="gimbalUwbFollow"/);
    assert.doesNotMatch(mobileHtml, /id="gimbalUwbFollowControl"/);
    assert.match(mobileHtml, /id="manualFollowButton"/);
    assert.match(mobileHtml, /id="autoFollowButton"/);
    assert.match(mobileHtml, /id="driveFollowControl"/);
    assert.match(mobileHtml, /id="driveFollowSwitch"/);
    assert.match(mobileHtml, /joystick\.hidden=automatic/);
    assert.match(mobileHtml, /gps\.fixType\|\|0\)<2/);
    assert.match(mobileHtml, /state\.ekf\.healthy/);
    assert.match(transportScript, /setDriveEnabled/);
    assert.match(transportScript, /\/api\/uwb-follow\/start/);
    assert.match(transportScript, /uwb_follow_update/);
    assert.match(serverScript, /sendMavlinkCommand\('SET_MODE', \{ mode: 'GUIDED'/);
    assert.match(serverScript, /sendMavlinkCommand\('ARM'/);
    assert.match(serverScript, /waitForFollowTelemetry/);
});

test("3D gimbal calibration does not overwrite the boat-follow bearing calibration", () => {
    assert.match(serverScript, /bearing_sign: GIMBAL_BEACON_CONFIG\.yaw_source_sign/);
    assert.match(serverScript, /bearing_scale: GIMBAL_BEACON_CONFIG\.yaw_source_scale/);
    assert.match(serverScript, /bearing_offset_deg: GIMBAL_BEACON_CONFIG\.yaw_offset_deg/);
    const start = serverScript.indexOf('function persistBeaconCalibration(result)');
    const end = serverScript.indexOf('function startBeaconCalibrationSession()', start);
    assert.doesNotMatch(serverScript.slice(start, end), /setBearingCalibration/);
});
