#!/usr/bin/env python3
'''PNU : log what the FC says, for an unattended SITL run.

WHY THIS EXISTS.  A run driven by run_all3.sh with no ground station attached leaves no
record of the FC side at all.  The dataflash log is not a substitute: the logger starts
about six seconds after boot, so the one line that matters most - "PNU_CAN: CAN_D1
PMU+Gimbal+SAR ready" - is already gone by the time it opens.  Two sessions were spent
re-running SITL to read a message that had been printed and thrown away.

It parses MAVLink2 by hand, as fake_sar.py parses CAN by hand, for two reasons.  The
first is that it has no dependencies, so it runs wherever the fakes run.  The second is
that pymavlink only knows the standard dialect, and the message this project most needs
to watch - TM 51050, SYS_ICD_FLCC_GCS_SAR_STATUS - is in the PNU dialect; pymavlink hands
it back as BAD_DATA.  Keeping the parser here keeps the tap honest about wire order:
Live_Counter is a uint32 at offset 0 and SAR_STATUS a uint8 at offset 4, which is NOT the
order the XML lists them in.

It sends a 1 Hz GCS heartbeat because ArduPilot only streams STATUSTEXT to a channel it
considers active, and a channel becomes active when traffic arrives on it.  A silent
listener hears nothing.

AP_FLAKE8_CLEAN
'''

import argparse
import socket
import struct
import sys
import time

MSG_HEARTBEAT = 0
MSG_STATUSTEXT = 253
MSG_TM50 = 51050            # SYS_ICD_FLCC_GCS_SAR_STATUS

SEVERITY = ('EMERG', 'ALERT', 'CRIT', 'ERROR', 'WARN', 'NOTICE', 'INFO', 'DEBUG')

SAR_STATE = {0: 'Disconnected', 1: 'Ready', 2: 'Stopped',
             3: 'Observing', 4: 'Error'}


# x25 CRC, the MAVLink checksum
def crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def crc_of(data, extra):
    crc = 0xFFFF
    for byte in data:
        crc = crc_accumulate(byte, crc)
    return crc_accumulate(extra, crc)


HEARTBEAT_CRC_EXTRA = 50


def heartbeat(seq):
    '''a GCS heartbeat: type 6 GCS, autopilot 8 INVALID, nothing else set'''
    payload = struct.pack('<IBBBBB', 0, 6, 8, 0, 0, 3)
    header = struct.pack('<BBBBBBBBH', 0xFD, len(payload), 0, 0,
                         seq & 0xFF, 255, 190, MSG_HEARTBEAT, 0)
    crc = crc_of(header[1:] + payload, HEARTBEAT_CRC_EXTRA)
    return header + payload + struct.pack('<H', crc)


class Tap(object):
    def __init__(self, url, out):
        self.url = url
        self.out = out
        self.buf = b''
        self.sock = None
        self.t0 = time.monotonic()
        self.last_tm50 = None
        self.last_hb = None
        self.statustexts = 0
        self.chunk_id = 0
        self.chunk_text = ''
        self.chunk_severity = 7

    def log(self, msg):
        self.out.write('%.3f [%8.3f] %s\n' % (time.time(),
                                              time.monotonic() - self.t0, msg))
        self.out.flush()

    def connect(self, timeout):
        '''keep trying: SITL may not have opened the port yet, and a tap that dies on
           the first ECONNREFUSED is worse than no tap, because the run looks fine'''
        kind, host, port = self.url.split(':')
        if kind != 'tcp':
            raise SystemExit('only tcp:host:port is supported, got %s' % self.url)
        deadline = time.monotonic() + timeout
        announced = False
        while time.monotonic() < deadline:
            try:
                sock = socket.create_connection((host, int(port)), 2.0)
                sock.settimeout(0.2)
                self.sock = sock
                self.log('connected to %s' % self.url)
                return True
            except OSError as err:
                if not announced:
                    announced = True
                    self.log('%s not up yet (%s) - retrying' % (self.url, err))
                # tight, because the tap is normally started before SITL and the
                # messages worth having are in the first half second after the port
                # opens - the gimbal's "roll enabled" among them
                time.sleep(0.1)
        self.log('gave up connecting to %s' % self.url)
        return False

    # -- the wire ----------------------------------------------------------------------
    def frames(self):
        '''yield (msgid, payload) for every complete MAVLink2 frame in the buffer

           No CRC check.  The tap has no CRC_EXTRA table for the PNU dialect and
           inventing one here would be a second place to get TM50 wrong; the length
           field is enough to resynchronise on a TCP stream that never loses bytes.'''
        while True:
            start = self.buf.find(b'\xfd')
            if start < 0:
                self.buf = b''
                return
            if len(self.buf) < start + 12:
                self.buf = self.buf[start:]
                return
            length = self.buf[start + 1]
            incompat = self.buf[start + 2]
            total = 12 + length + (13 if (incompat & 0x01) else 0)
            if len(self.buf) < start + total:
                self.buf = self.buf[start:]
                return
            frame = self.buf[start:start + total]
            self.buf = self.buf[start + total:]
            msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            yield msgid, frame[10:10 + length]

    def emit_statustext(self, severity, text):
        self.statustexts += 1
        self.log('%-6s %s' % (SEVERITY[severity] if severity < 8 else '?', text))

    def flush_chunks(self):
        if self.chunk_id:
            self.emit_statustext(self.chunk_severity, self.chunk_text)
            self.chunk_id = 0
            self.chunk_text = ''

    def handle_statustext(self, payload):
        '''reassemble the chunks of a message longer than 50 characters

           GCS_Common.cpp:2564-2592 splits anything over 50 characters into 50-byte
           chunks that share a non-zero `id` and carry an increasing `chunk_seq`, and
           null-terminates the last one.  Logging each chunk as its own line reads
           exactly like a firmware bug - it is how this tap first reported two gimbal
           messages as "truncated on the wire" when nothing of the sort was happening.'''
        severity = payload[0] if payload else 7
        field = payload[1:51]
        text = field.split(b'\x00')[0].decode('ascii', 'replace')
        # MAVLink2 drops EVERY trailing zero byte, so a chunk whose id is below 256
        # arrives with the id's high byte and chunk_seq cut off and the payload 52 bytes
        # long, not 54.  Reading the extensions only when all three bytes are present
        # saw msg_id 0 for every chunk and logged the halves as separate messages.
        ext = payload[51:]
        ext = ext + b'\x00' * (3 - len(ext)) if len(ext) < 3 else ext
        msg_id, chunk_seq = struct.unpack('<HB', ext[:3])

        if msg_id == 0:
            self.flush_chunks()
            self.emit_statustext(severity, text)
            return
        if chunk_seq == 0 or msg_id != self.chunk_id:
            self.flush_chunks()
            self.chunk_id = msg_id
            self.chunk_text = ''
            self.chunk_severity = severity
        self.chunk_text += text
        # a chunk with room to spare is the last one; a full 50 means more is coming
        if len(field) < 50 or b'\x00' in field:
            self.flush_chunks()

    def handle(self, msgid, payload):
        if msgid == MSG_STATUSTEXT:
            self.handle_statustext(payload)
        elif msgid == MSG_TM50:
            # zero-trimmed payloads are legal in MAVLink2, so pad before unpacking
            data = payload + b'\x00' * (5 - len(payload)) if len(payload) < 5 else payload
            live, status = struct.unpack('<IB', data[:5])
            # only on a change of state: TM50 goes out at 5 Hz unconditionally and
            # Live_Counter moves every time, so logging each one buries everything else
            if status != self.last_tm50:
                self.last_tm50 = status
                self.log('TM50   SAR_STATUS=%u %-12s Live_Counter=%u'
                         % (status, SAR_STATE.get(status, '?'), live))
        elif msgid == MSG_HEARTBEAT and len(payload) >= 9:
            custom, _, _, base, _ = struct.unpack('<IBBBB', payload[:8])
            armed = bool(base & 0x80)
            if (custom, armed) != self.last_hb:
                self.last_hb = (custom, armed)
                self.log('HB     mode=%u armed=%s' % (custom, armed))

    def run(self, duration):
        seq = 0
        next_beat = 0.0
        connected_at = time.monotonic()
        end = connected_at + duration if duration > 0 else None
        while end is None or time.monotonic() < end:
            now = time.monotonic()
            if now >= next_beat:
                # fast at first.  The FC streams STATUSTEXT only to a channel it has
                # heard from, and when the tap is started before SITL it can attach
                # before the FC has read anything - one heartbeat a second then loses
                # the boot messages, which are the ones worth having.
                next_beat = now + (0.2 if now - connected_at < 3.0 else 1.0)
                try:
                    self.sock.sendall(heartbeat(seq))
                    seq += 1
                except OSError as err:
                    self.log('link lost on send: %s' % err)
                    return
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as err:
                self.log('link lost on recv: %s' % err)
                return
            if not chunk:
                self.log('link closed by SITL')
                return
            self.buf += chunk
            for msgid, payload in self.frames():
                self.handle(msgid, payload)


def main():
    parser = argparse.ArgumentParser(description='log FC statustexts and TM50 from SITL')
    parser.add_argument('url', nargs='?', default='tcp:127.0.0.1:5765')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds to run, 0 means until the link closes')
    parser.add_argument('--connect-timeout', type=float, default=60.0)
    args = parser.parse_args()

    tap = Tap(args.url, sys.stdout)
    if not tap.connect(args.connect_timeout):
        return 1
    try:
        tap.run(args.duration)
    except KeyboardInterrupt:
        pass
    tap.flush_chunks()
    tap.log('%u statustexts' % tap.statustexts)
    return 0


if __name__ == '__main__':
    sys.exit(main())
