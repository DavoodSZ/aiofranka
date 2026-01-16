# Web Terminal Setup Guide

This guide documents how to expose a terminal over the web using Python, WebSocket, and nginx reverse proxy with SSL.

## Architecture Overview

```
[Browser] <--HTTPS/WSS--> [nginx] <--HTTP/WS--> [Python Server] <--PTY--> [Shell]
                              |
                              v
                         [Let's Encrypt SSL]
```

## Prerequisites

- A server with a public-facing domain (e.g., `yourserver.example.com`)
- Python 3.8+
- nginx
- certbot (for SSL)

## Step 1: Install Dependencies

```bash
# System packages
sudo apt update
sudo apt install nginx certbot python3-certbot-nginx

# Python packages
pip install aiohttp
```

## Step 2: Create the Python WebSocket Terminal Server

Create `server.py`:

```python
import os
import pty
import asyncio
import fcntl
import struct
import termios
import signal
from aiohttp import web

class WebTerminal:
    """PTY-based terminal that communicates over WebSocket."""
    
    def __init__(self):
        self.master_fd = None
        self.pid = None
    
    def spawn(self, shell='/bin/bash'):
        """Spawn a new PTY with shell."""
        self.pid, self.master_fd = pty.fork()
        
        if self.pid == 0:
            # Child process - exec shell
            env = os.environ.copy()
            env['TERM'] = 'xterm-256color'
            os.execvpe(shell, [shell], env)
        else:
            # Parent - set non-blocking
            flags = fcntl.fcntl(self.master_fd, fcntl.F_GETFL)
            fcntl.fcntl(self.master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    
    def resize(self, rows, cols):
        """Resize the PTY."""
        if self.master_fd:
            winsize = struct.pack('HHHH', rows, cols, 0, 0)
            fcntl.ioctl(self.master_fd, termios.TIOCSWINSZ, winsize)
    
    def write(self, data):
        """Write data to PTY."""
        if self.master_fd:
            os.write(self.master_fd, data.encode())
    
    def read(self):
        """Read available data from PTY."""
        if self.master_fd:
            try:
                return os.read(self.master_fd, 4096).decode('utf-8', errors='replace')
            except (OSError, BlockingIOError):
                return None
        return None
    
    def close(self):
        """Clean up PTY."""
        if self.master_fd:
            os.close(self.master_fd)
        if self.pid:
            try:
                os.kill(self.pid, signal.SIGTERM)
                os.waitpid(self.pid, 0)
            except:
                pass


async def terminal_handler(request):
    """WebSocket handler for terminal connections."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    terminal = WebTerminal()
    terminal.spawn()
    
    # Read loop - send PTY output to WebSocket
    async def read_loop():
        while not ws.closed:
            output = terminal.read()
            if output:
                await ws.send_str(output)
            await asyncio.sleep(0.01)
    
    read_task = asyncio.create_task(read_loop())
    
    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                data = msg.json() if msg.data.startswith('{') else {'type': 'input', 'data': msg.data}
                
                if data.get('type') == 'resize':
                    terminal.resize(data.get('rows', 24), data.get('cols', 80))
                elif data.get('type') == 'input':
                    terminal.write(data.get('data', ''))
                else:
                    terminal.write(msg.data)
    finally:
        read_task.cancel()
        terminal.close()
    
    return ws


async def index_handler(request):
    """Serve the terminal web page."""
    html = '''<!DOCTYPE html>
<html>
<head>
    <title>Web Terminal</title>
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/xterm@5.3.0/css/xterm.min.css">
    <script src="https://cdn.jsdelivr.net/npm/xterm@5.3.0/lib/xterm.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/xterm-addon-fit@0.8.0/lib/xterm-addon-fit.min.js"></script>
    <style>
        body { margin: 0; padding: 20px; background: #1e1e1e; }
        #terminal { height: calc(100vh - 40px); }
    </style>
</head>
<body>
    <div id="terminal"></div>
    <script>
        const term = new Terminal({
            cursorBlink: true,
            fontSize: 14,
            fontFamily: 'Menlo, Monaco, "Courier New", monospace',
            theme: { background: '#1e1e1e' }
        });
        const fitAddon = new FitAddon.FitAddon();
        term.loadAddon(fitAddon);
        term.open(document.getElementById('terminal'));
        fitAddon.fit();
        
        // Connect WebSocket
        const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws = new WebSocket(`${protocol}//${location.host}/terminal`);
        
        ws.onopen = () => {
            // Send initial size
            ws.send(JSON.stringify({type: 'resize', rows: term.rows, cols: term.cols}));
        };
        
        ws.onmessage = (e) => term.write(e.data);
        ws.onclose = () => term.write('\\r\\n[Connection closed]\\r\\n');
        
        term.onData((data) => ws.send(JSON.stringify({type: 'input', data: data})));
        
        window.addEventListener('resize', () => {
            fitAddon.fit();
            ws.send(JSON.stringify({type: 'resize', rows: term.rows, cols: term.cols}));
        });
    </script>
</body>
</html>'''
    return web.Response(text=html, content_type='text/html')


# Create app
app = web.Application()
app.router.add_get('/', index_handler)
app.router.add_get('/terminal', terminal_handler)

if __name__ == '__main__':
    web.run_app(app, host='127.0.0.1', port=8890)
```

## Step 3: Set Up SSL with Let's Encrypt

```bash
# Get SSL certificate
sudo certbot --nginx -d yourserver.example.com

# This will:
# 1. Verify domain ownership
# 2. Generate certificates in /etc/letsencrypt/live/yourserver.example.com/
# 3. Auto-configure nginx for HTTPS
```

## Step 4: Configure nginx as Reverse Proxy

Create `/etc/nginx/sites-available/yourserver.example.com`:

```nginx
server {
    server_name yourserver.example.com;

    # Proxy to Python server
    location / {
        proxy_pass http://127.0.0.1:8890/;
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        
        # CRITICAL for WebSocket support
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        
        # Long timeout for WebSocket connections
        proxy_read_timeout 86400;
        proxy_connect_timeout 60;
    }

    # SSL config (added by certbot)
    listen 443 ssl;
    ssl_certificate /etc/letsencrypt/live/yourserver.example.com/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/yourserver.example.com/privkey.pem;
    include /etc/letsencrypt/options-ssl-nginx.conf;
    ssl_dhparam /etc/letsencrypt/ssl-dhparams.pem;
}

# HTTP to HTTPS redirect
server {
    listen 80;
    server_name yourserver.example.com;
    return 301 https://$host$request_uri;
}
```

Enable the site:

```bash
sudo ln -s /etc/nginx/sites-available/yourserver.example.com /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl reload nginx
```

## Step 5: Run the Server

```bash
# Run directly
python server.py

# Or with systemd service (see below)
```

## Step 6 (Optional): Create systemd Service

Create `/etc/systemd/system/web-terminal.service`:

```ini
[Unit]
Description=Web Terminal Server
After=network.target

[Service]
Type=simple
User=yourusername
WorkingDirectory=/path/to/server
ExecStart=/usr/bin/python3 /path/to/server/server.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable web-terminal
sudo systemctl start web-terminal
```

## Step 7 (Recommended): Add Authentication

### Option A: nginx Basic Auth

```bash
# Create password file
sudo apt install apache2-utils
sudo htpasswd -c /etc/nginx/.htpasswd yourusername
```

Add to nginx location block:

```nginx
location / {
    auth_basic "Terminal Access";
    auth_basic_user_file /etc/nginx/.htpasswd;
    
    proxy_pass http://127.0.0.1:8890/;
    # ... rest of config
}
```

### Option B: Token-based Auth in Python

```python
import secrets

VALID_TOKEN = secrets.token_urlsafe(32)
print(f"Access token: {VALID_TOKEN}")

async def terminal_handler(request):
    # Check token in query string
    token = request.query.get('token')
    if token != VALID_TOKEN:
        return web.Response(status=403, text='Invalid token')
    
    # ... rest of handler
```

## Security Considerations

1. **Always use HTTPS** - Never expose terminal over plain HTTP
2. **Add authentication** - Don't leave terminals publicly accessible
3. **Restrict by IP** - Use nginx `allow`/`deny` if possible:
   ```nginx
   allow 10.0.0.0/8;    # Internal network
   deny all;
   ```
4. **Use non-root user** - The shell runs as the Python server's user
5. **Consider session limits** - Add timeouts or connection limits

## Troubleshooting

### WebSocket connection fails

1. Check nginx has WebSocket headers:
   ```nginx
   proxy_set_header Upgrade $http_upgrade;
   proxy_set_header Connection "upgrade";
   ```

2. Check firewall allows ports 80/443:
   ```bash
   sudo ufw allow 80
   sudo ufw allow 443
   ```

### Terminal not responding

1. Check PTY is spawned correctly:
   ```bash
   ps aux | grep bash
   ```

2. Check server logs:
   ```bash
   journalctl -u web-terminal -f
   ```

### SSL certificate issues

```bash
# Test certificate
sudo certbot renew --dry-run

# Force renewal
sudo certbot renew --force-renewal
```

## Files Summary

| File | Purpose |
|------|---------|
| `server.py` | Python WebSocket terminal server |
| `/etc/nginx/sites-available/...` | nginx reverse proxy config |
| `/etc/letsencrypt/live/.../` | SSL certificates |
| `/etc/systemd/system/web-terminal.service` | Auto-start service |

## Quick Start Checklist

- [ ] Install dependencies (`aiohttp`, nginx, certbot)
- [ ] Create `server.py` with terminal handler
- [ ] Get SSL certificate with certbot
- [ ] Configure nginx with WebSocket support
- [ ] Add authentication
- [ ] Create systemd service for auto-start
- [ ] Test from browser
