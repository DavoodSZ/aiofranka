#!/usr/bin/env python3
"""
Low-latency WebRTC camera streaming server with web terminal.
Streams cameras via WebRTC for minimal latency across long distances.
Includes a web-based terminal for remote command execution.
"""

import asyncio
import json
import logging
import cv2
import numpy as np
from aiohttp import web
import aiohttp
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame
import fractions
import time
import threading
import pty
import os
import select
import struct
import fcntl
import termios

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("camera-stream")

# Configuration - Two RealSense cameras
CAMERAS = {
    "cam1": {"device": "/dev/video4", "name": "Camera 1 (USB-2)"},
    "cam2": {"device": "/dev/video10", "name": "Camera 2 (USB-8)"},
}
WIDTH = 640
HEIGHT = 480
FPS = 30

pcs = set()
camera_readers = {}


class CameraReader:
    """Thread-safe camera reader that runs in a background thread."""
    
    def __init__(self, device: str, width: int = 640, height: int = 480, fps: int = 30):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        
    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info(f"Started camera reader for {self.device}")
        
    def _capture_loop(self):
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        logger.info(f"Opened camera {self.device}")
        
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame.copy()
            else:
                time.sleep(0.001)
                
        self.cap.release()
        
    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)


class CameraVideoTrack(VideoStreamTrack):
    """Video track that reads from a thread-safe camera reader."""
    
    kind = "video"
    
    def __init__(self, reader: CameraReader):
        super().__init__()
        self.reader = reader
        self._timestamp = 0
        self._time_base = fractions.Fraction(1, 90000)
        self._start_time = time.time()
        self._frame_count = 0
        self._lock = asyncio.Lock()
        
    async def recv(self):
        async with self._lock:
            # Frame pacing
            elapsed = time.time() - self._start_time
            expected_frame = int(elapsed * self.reader.fps)
            if self._frame_count > expected_frame:
                await asyncio.sleep(1.0 / self.reader.fps)
            
            # Get frame from reader
            frame = self.reader.get_frame()
            self._frame_count += 1
            
            if self._frame_count % 30 == 0:
                logger.info(f"Sent {self._frame_count} frames")
            
            if frame is None:
                frame = np.zeros((self.reader.height, self.reader.width, 3), dtype=np.uint8)
            
            # Handle depth camera (16-bit) - colorize it
            if frame.dtype == np.uint16:
                frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
                frame = frame.astype(np.uint8)
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            elif len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            # Convert BGR to RGB for WebRTC
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Ensure frame is contiguous in memory
            frame = np.ascontiguousarray(frame)
            
            # Create video frame
            video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
            video_frame.pts = self._timestamp
            video_frame.time_base = self._time_base
            self._timestamp += int(90000 / self.reader.fps)
            
            return video_frame
    
    def stop(self):
        super().stop()


def get_camera_reader(cam_id: str):
    """Get or create a camera reader for the specified camera."""
    global camera_readers
    if cam_id not in camera_readers:
        if cam_id not in CAMERAS:
            return None
        camera_readers[cam_id] = CameraReader(CAMERAS[cam_id]["device"], WIDTH, HEIGHT, FPS)
        camera_readers[cam_id].start()
    return camera_readers[cam_id]


async def index(request):
    """Serve the main page with video players."""
    content = """
<!DOCTYPE html>
<html>
<head>
    <title>Franka Camera Live Feed</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #1a1a2e;
            color: #eee;
            margin: 0;
            padding: 20px;
        }
        h1 {
            text-align: center;
            color: #00d4ff;
        }
        .container {
            display: flex;
            flex-wrap: wrap;
            justify-content: center;
            gap: 20px;
            max-width: 1400px;
            margin: 0 auto;
        }
        .camera-box {
            background: #16213e;
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
        }
        .camera-box h3 {
            margin: 0 0 10px 0;
            color: #00d4ff;
        }
        video {
            width: 640px;
            height: 480px;
            background: #000;
            border-radius: 8px;
        }
        .status {
            margin-top: 10px;
            padding: 5px 10px;
            border-radius: 4px;
            font-size: 12px;
        }
        .status.connected { background: #00c853; color: #000; }
        .status.connecting { background: #ffab00; color: #000; }
        .status.error { background: #ff1744; }
        .stats {
            font-size: 11px;
            color: #888;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <h1>🤖 Franka Robot - Live Camera Feed</h1>
    <div class="container">
        <div class="camera-box">
            <h3>📷 Camera 1</h3>
            <video id="video1" autoplay playsinline muted></video>
            <div id="status1" class="status connecting">Connecting...</div>
            <div id="stats1" class="stats"></div>
        </div>
        <div class="camera-box">
            <h3>📷 Camera 2</h3>
            <video id="video2" autoplay playsinline muted></video>
            <div id="status2" class="status connecting">Connecting...</div>
            <div id="stats2" class="stats"></div>
        </div>
    </div>
    
    <script>
        const cameras = ['cam1', 'cam2'];
        const pcs = {};
        
        async function connectCamera(camId, videoId, statusId, statsId) {
            const video = document.getElementById(videoId);
            const status = document.getElementById(statusId);
            const stats = document.getElementById(statsId);
            
            if (pcs[camId]) { pcs[camId].close(); pcs[camId] = null; }
            
            status.textContent = 'Connecting...';
            status.className = 'status connecting';
            
            try {
                const pc = new RTCPeerConnection({
                    iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
                });
                pcs[camId] = pc;
                
                pc.ontrack = (event) => {
                    video.srcObject = event.streams[0];
                    status.textContent = 'Connected';
                    status.className = 'status connected';
                };
                
                pc.oniceconnectionstatechange = () => {
                    if (pc.iceConnectionState === 'failed' || pc.iceConnectionState === 'disconnected') {
                        status.textContent = 'Reconnecting...';
                        status.className = 'status error';
                        setTimeout(() => connectCamera(camId, videoId, statusId, statsId), 3000);
                    }
                };
                
                pc.addTransceiver('video', { direction: 'recvonly' });
                const offer = await pc.createOffer();
                await pc.setLocalDescription(offer);
                
                await new Promise(r => {
                    if (pc.iceGatheringState === 'complete') r();
                    else { pc.onicegatheringstatechange = () => { if (pc.iceGatheringState === 'complete') r(); }; setTimeout(r, 3000); }
                });
                
                const basePath = window.location.pathname.endsWith('/') ? window.location.pathname : window.location.pathname + '/';
                const response = await fetch(basePath + 'offer/' + camId, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type })
                });
                
                const answer = await response.json();
                await pc.setRemoteDescription(new RTCSessionDescription(answer));
                
                setInterval(async () => {
                    if (pcs[camId]) {
                        const report = await pcs[camId].getStats();
                        report.forEach(s => {
                            if (s.type === 'inbound-rtp' && s.kind === 'video') {
                                stats.textContent = (s.framesPerSecond || 0).toFixed(1) + ' fps';
                            }
                        });
                    }
                }, 1000);
            } catch (e) {
                console.error(e);
                status.textContent = 'Error - Reconnecting...';
                status.className = 'status error';
                setTimeout(() => connectCamera(camId, videoId, statusId, statsId), 3000);
            }
        }
        
        connectCamera('cam1', 'video1', 'status1', 'stats1');
        connectCamera('cam2', 'video2', 'status2', 'stats2');
    </script>
</body>
</html>
"""
    return web.Response(content_type="text/html", text=content)


async def offer(request):
    """Handle WebRTC offer from client."""
    cam_id = request.match_info.get('cam_id', 'cam1')
    params = await request.json()
    
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    
    pc = RTCPeerConnection()
    pcs.add(pc)
    
    logger.info(f"New connection for {cam_id}, total: {len(pcs)}")
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info(f"Connection state ({cam_id}): {pc.connectionState}")
        if pc.connectionState in ("failed", "closed", "disconnected"):
            await pc.close()
            pcs.discard(pc)
            logger.info(f"Connection closed, remaining: {len(pcs)}")
    
    # Create a new track for this connection (shares the camera reader)
    reader = get_camera_reader(cam_id)
    if reader is None:
        return web.json_response({"error": f"Unknown camera: {cam_id}"}, status=404)
    
    track = CameraVideoTrack(reader)
    pc.addTrack(track)
    
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })


async def on_shutdown(app):
    """Cleanup on shutdown."""
    global camera_readers
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()
    
    for reader in camera_readers.values():
        reader.stop()
    camera_readers.clear()


async def inject_js(request):
    """Serve the injection script for Franka Desk."""
    script_path = "/home/younghyo/camera-stream/inject.js"
    with open(script_path, "r") as f:
        content = f.read()
    return web.Response(content_type="application/javascript", text=content)


class WebTerminal:
    """Manages a PTY terminal session over WebSocket."""
    
    def __init__(self, ws):
        self.ws = ws
        self.fd = None
        self.pid = None
        self.running = False
        
    async def start(self):
        """Start the terminal session."""
        # Fork a PTY
        self.pid, self.fd = pty.fork()
        
        if self.pid == 0:
            # Child process - exec shell
            os.environ['TERM'] = 'xterm-256color'
            os.environ['COLORTERM'] = 'truecolor'
            os.chdir(os.path.expanduser('~'))
            os.execlp('/bin/zsh', 'zsh', '-l')
        else:
            # Parent process
            self.running = True
            # Set non-blocking
            flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            
            # Start reading task
            asyncio.create_task(self._read_output())
            
    async def _read_output(self):
        """Read output from PTY and send to WebSocket."""
        loop = asyncio.get_event_loop()
        while self.running:
            try:
                # Check if there's data to read
                r, _, _ = select.select([self.fd], [], [], 0.01)
                if r:
                    data = os.read(self.fd, 4096)
                    if data:
                        await self.ws.send_str(data.decode('utf-8', errors='replace'))
                else:
                    await asyncio.sleep(0.01)
            except (OSError, IOError):
                break
            except Exception as e:
                logger.error(f"Terminal read error: {e}")
                break
        
    def write(self, data: str):
        """Write input to the PTY."""
        if self.fd:
            try:
                os.write(self.fd, data.encode('utf-8'))
            except (OSError, IOError) as e:
                logger.error(f"Terminal write error: {e}")
                
    def resize(self, rows: int, cols: int):
        """Resize the PTY."""
        if self.fd:
            try:
                winsize = struct.pack('HHHH', rows, cols, 0, 0)
                fcntl.ioctl(self.fd, termios.TIOCSWINSZ, winsize)
            except (OSError, IOError) as e:
                logger.error(f"Terminal resize error: {e}")
                
    def stop(self):
        """Stop the terminal session."""
        self.running = False
        if self.fd:
            try:
                os.close(self.fd)
            except:
                pass
        if self.pid:
            try:
                os.kill(self.pid, 9)
                os.waitpid(self.pid, 0)
            except:
                pass


async def terminal_ws(request):
    """WebSocket handler for terminal."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    logger.info("Terminal WebSocket connected")
    terminal = WebTerminal(ws)
    await terminal.start()
    
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                data = json.loads(msg.data)
                if data.get('type') == 'input':
                    terminal.write(data.get('data', ''))
                elif data.get('type') == 'resize':
                    terminal.resize(data.get('rows', 24), data.get('cols', 80))
            elif msg.type == aiohttp.WSMsgType.ERROR:
                logger.error(f"Terminal WebSocket error: {ws.exception()}")
    finally:
        terminal.stop()
        logger.info("Terminal WebSocket disconnected")
    
    return ws


def main():
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/inject.js", inject_js)
    app.router.add_get("/terminal", terminal_ws)
    app.router.add_post("/offer/{cam_id}", offer)
    
    logger.info("Starting camera stream server on http://localhost:8890")
    web.run_app(app, host="0.0.0.0", port=8890)


if __name__ == "__main__":
    main()
