-- Frozen pre-shared-clock sender for compatibility regression tests only.
-- Dual ALX AOA UWB reader for Pixhawk TELEM1 and TELEM2.
-- Set SERIAL1_PROTOCOL=28, SERIAL1_BAUD=115, SERIAL2_PROTOCOL=28,
-- SERIAL2_BAUD=115. Receiver mount compensation is configured on the RK3588.
-- Telemetry only: this script does not control the vehicle or feed the EKF.

local MAV_SEVERITY = {ERROR = 3, WARNING = 4, INFO = 6}
local POSITION_LENGTH = 37
local HEARTBEAT_LENGTH = 16
-- Pixhawk1 has a tight Lua instruction budget.  Drain only one receiver per
-- scheduler tick, alternating at 100 Hz. Read at most 48 bytes per visit,
-- stopping at the first complete position. This supports 1850 B/s of clean
-- position frames per UART (50 * 37), with room to resynchronise on noise.
local MAX_BYTES_PER_RECEIVER = 48
-- No wall-clock throttle on position forwarding. At most one complete
-- position per receiver visit keeps each sample's MAVLink timestamp distinct
-- and bounds Lua work. Alternating 10 ms visits support up to 50 Hz/receiver.
local REPORT_PERIOD_MS = 1000
local NO_FRAME_PERIOD_MS = 5000
local MIN_DISTANCE_M = 0.15
local MAX_DISTANCE_M = 20.0

local function new_receiver(receiver_id, serial_instance)
    local uart = serial:find_serial(serial_instance)
    if not uart then
        gcs:send_text(
            MAV_SEVERITY.ERROR,
            string.format("UWB%d: scripting serial %d missing", receiver_id, serial_instance)
        )
        return nil
    end
    uart:begin(115200)
    uart:set_flow_control(0)
    return {
        id = receiver_id,
        uart = uart,
        frame = {},
        frame_index = 1,
        expected_length = 0,
        header_count = 0,
        collecting = false,
        valid_count = 0,
        rx_bytes = 0,
        checksum_errors = 0,
        length_errors = 0,
        heartbeat_count = 0,
        last_sequence_id = 0,
        last_anchor_id = 0,
        last_tag_id = 0,
        last_tag_status = 0,
        sent_count = 0,
        validity = nil,
        report_index = 0,
        last_warning_ms = 0,
        names = {},
        last_report_ms = 0,
        last_valid_ms = 0,
        max_available = 0,
    }
end

local receivers = {
    new_receiver(1, 0),
    new_receiver(2, 1),
}

if not receivers[1] and not receivers[2] then
    gcs:send_text(MAV_SEVERITY.ERROR, "UWB: no scripting serial ports")
    return
end

local function reset_parser(receiver)
    receiver.frame = {}
    receiver.frame_index = 1
    receiver.expected_length = 0
    receiver.header_count = 0
    receiver.collecting = false
end

local function u16(receiver, index)
    return receiver.frame[index] * 256 + receiver.frame[index + 1]
end

local function i16(receiver, index)
    local value = u16(receiver, index)
    if value >= 32768 then
        return value - 65536
    end
    return value
end

local function u32(receiver, index)
    return (((receiver.frame[index] * 256 + receiver.frame[index + 1]) * 256
        + receiver.frame[index + 2]) * 256 + receiver.frame[index + 3])
end

local function send_value(receiver, suffix, value)
    local name = receiver.names[suffix]
    if not name then
        name = string.format("UWB%d_%s", receiver.id, suffix)
        receiver.names[suffix] = name
    end
    gcs:send_named_float(name, value)
end

local function set_validity(receiver, valid)
    if receiver.validity ~= valid then
        send_value(receiver, "VALID", valid and 1 or 0)
        receiver.validity = valid
    end
end

local function process_frame(receiver, now_ms)
    local command = u16(receiver, 9)
    if command == 0x2002 and receiver.expected_length == HEARTBEAT_LENGTH then
        receiver.heartbeat_count = receiver.heartbeat_count + 1
        receiver.last_sequence_id = u16(receiver, 7)
        receiver.last_anchor_id = u32(receiver, 13)
        return
    end
    if command ~= 0x2001 or receiver.expected_length ~= POSITION_LENGTH then
        return
    end

    local checksum = 0
    for index = 1, POSITION_LENGTH - 1 do
        checksum = checksum ~ receiver.frame[index]
    end
    if checksum ~= receiver.frame[POSITION_LENGTH] then
        receiver.checksum_errors = receiver.checksum_errors + 1
        return
    end

    local distance_m = u32(receiver, 21) / 100.0
    local azimuth_deg = i16(receiver, 25)
    local elevation_deg = i16(receiver, 27)
    local sequence_id = u16(receiver, 7)
    local anchor_id = u32(receiver, 13)
    local tag_id = u32(receiver, 17)
    local tag_status = u16(receiver, 29)
    receiver.last_sequence_id = sequence_id
    receiver.last_anchor_id = anchor_id
    receiver.last_tag_id = tag_id
    receiver.last_tag_status = tag_status

    if distance_m < MIN_DISTANCE_M or distance_m > MAX_DISTANCE_M then
        set_validity(receiver, false)
        return true
    end
    receiver.valid_count = receiver.valid_count + 1
    receiver.last_valid_ms = now_ms

    -- Three floats per position, rather than five. GOOD and other counters
    -- are diagnostics; send them separately so two 40 Hz receivers leave
    -- room on a 115200 baud MAVLink link for attitude and motor feedback.
    -- AZ completes the pair in the bridge. Send elevation and any validity
    -- transition before AZ, so the published observation is already coherent.
    send_value(receiver, "DIST", distance_m)
    send_value(receiver, "EL", elevation_deg)
    set_validity(receiver, true)
    send_value(receiver, "AZ", azimuth_deg)
    receiver.sent_count = receiver.sent_count + 1
    return true
end

local function consume_byte(receiver, value, now_ms)
    if not receiver.collecting then
        if value == 0xFF then
            receiver.header_count = receiver.header_count + 1
            if receiver.header_count == 4 then
                receiver.frame = {0xFF, 0xFF, 0xFF, 0xFF}
                receiver.frame_index = 5
                receiver.collecting = true
            end
        else
            receiver.header_count = 0
        end
        return
    end

    receiver.frame[receiver.frame_index] = value
    receiver.frame_index = receiver.frame_index + 1

    if receiver.frame_index == 7 then
        receiver.expected_length = u16(receiver, 5)
        if receiver.expected_length ~= POSITION_LENGTH
            and receiver.expected_length ~= HEARTBEAT_LENGTH then
            receiver.length_errors = receiver.length_errors + 1
            reset_parser(receiver)
            return
        end
    end

    if receiver.expected_length > 0 and receiver.frame_index > receiver.expected_length then
        local position_complete = process_frame(receiver, now_ms)
        reset_parser(receiver)
        return position_complete
    end
end

local function update_receiver(receiver, now_ms)
    if not receiver then
        return
    end
    local available = receiver.uart:available():toint()
    if available > receiver.max_available then
        receiver.max_available = available
    end
    local read_count = math.min(available, MAX_BYTES_PER_RECEIVER)
    for _ = 1, read_count do
        local value = receiver.uart:read()
        if value >= 0 then
            receiver.rx_bytes = receiver.rx_bytes + 1
            if consume_byte(receiver, value, now_ms) then
                break
            end
        end
    end

    -- Spread diagnostics over visits, with at most one extra message per
    -- visit. A burst of diagnostics plus a position can exhaust Pixhawk1's
    -- Lua instruction budget and MAVLink TX space.
    if receiver.report_index == 0 and now_ms - receiver.last_report_ms >= REPORT_PERIOD_MS then
        receiver.report_index = 1
        receiver.last_report_ms = now_ms
    end
    local index = receiver.report_index
    if index == 1 then send_value(receiver, "GOOD", receiver.valid_count)
    elseif index == 2 then send_value(receiver, "SENT", receiver.sent_count)
    elseif index == 3 then send_value(receiver, "RX", receiver.rx_bytes)
    elseif index == 4 then send_value(receiver, "HB", receiver.heartbeat_count)
    elseif index == 5 then send_value(receiver, "CRC", receiver.checksum_errors)
    elseif index == 6 then send_value(receiver, "LEN", receiver.length_errors)
    elseif index == 7 then send_value(receiver, "SEQ", receiver.last_sequence_id)
    elseif index == 8 then send_value(receiver, "ANCH", receiver.last_anchor_id)
    elseif index == 9 then send_value(receiver, "TAG", receiver.last_tag_id)
    elseif index == 10 then send_value(receiver, "STAT", receiver.last_tag_status)
    elseif index == 11 then
        send_value(receiver, "AVL", receiver.max_available)
        receiver.max_available = 0
    elseif index == 12 then
        -- Periodic state refresh recovers from a lost transition packet or
        -- bridge reconnect. An old validity flag cannot revive stale data.
        receiver.validity = receiver.valid_count > 0
            and now_ms - receiver.last_valid_ms < REPORT_PERIOD_MS
        send_value(receiver, "VALID", receiver.validity and 1 or 0)
    end
    if index > 0 then receiver.report_index = index < 12 and index + 1 or 0 end

    if now_ms - receiver.last_warning_ms >= NO_FRAME_PERIOD_MS
        and (receiver.valid_count == 0 or now_ms - receiver.last_valid_ms >= NO_FRAME_PERIOD_MS) then
        set_validity(receiver, false)
        gcs:send_text(MAV_SEVERITY.WARNING,
            string.format("UWB%d: no frame rx=%u good=%u crc=%u len=%u",
                receiver.id, receiver.rx_bytes, receiver.valid_count,
                receiver.checksum_errors, receiver.length_errors))
        receiver.last_warning_ms = now_ms
    end
end

local next_receiver = 1
local function update()
    local now_ms = millis():tofloat()
    update_receiver(receivers[next_receiver], now_ms)
    next_receiver = 3 - next_receiver
    return update, 10
end

gcs:send_text(MAV_SEVERITY.INFO, "UWB: per-frame TELEM1+TELEM2 readers started")
return update()
