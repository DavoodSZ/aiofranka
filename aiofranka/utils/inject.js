// Camera feed and terminal injection script for Franka Desk
(function() {
    'use strict';
    
    // Don't inject on login page
    if (window.location.pathname.startsWith('/login')) {
        return;
    }
    
    // Load xterm.js CSS and JS
    function loadXterm(callback) {
        // CSS
        const css = document.createElement('link');
        css.rel = 'stylesheet';
        css.href = 'https://cdn.jsdelivr.net/npm/xterm@5.3.0/css/xterm.min.css';
        document.head.appendChild(css);
        
        // JS
        const script = document.createElement('script');
        script.src = 'https://cdn.jsdelivr.net/npm/xterm@5.3.0/lib/xterm.min.js';
        script.onload = () => {
            // Load fit addon
            const fitScript = document.createElement('script');
            fitScript.src = 'https://cdn.jsdelivr.net/npm/xterm-addon-fit@0.8.0/lib/xterm-addon-fit.min.js';
            fitScript.onload = callback;
            document.head.appendChild(fitScript);
        };
        document.head.appendChild(script);
    }
    
    // Wait for the page to load
    function injectCameraFeed() {
        // Check if already injected
        if (document.getElementById('camera-feed-container')) return;
        
        // Create camera container
        const container = document.createElement('div');
        container.id = 'camera-feed-container';
        container.style.cssText = `
            position: fixed;
            top: 60px;
            left: 10px;
            z-index: 9999;
            display: flex;
            flex-direction: column;
            gap: 10px;
        `;
        
        // Create camera boxes
        const cameras = [
            { id: 'cam1', name: 'Camera 1' },
            { id: 'cam2', name: 'Camera 2' }
        ];
        
        cameras.forEach((cam, index) => {
            const box = document.createElement('div');
            box.id = 'cam-box-' + cam.id;
            box.className = 'camera-box-draggable';
            box.style.cssText = `
                position: absolute;
                top: 80px;
                left: ${20 + index * 530}px;
                background: rgba(22, 33, 62, 0.95);
                border-radius: 8px;
                padding: 8px;
                box-shadow: 0 4px 15px rgba(0,0,0,0.3);
                cursor: move;
                width: 510px;
                resize: both;
                overflow: hidden;
                border: 1px solid #00d4ff33;
            `;
            
            const header = document.createElement('div');
            header.className = 'drag-handle';
            header.style.cssText = `
                display: flex;
                justify-content: space-between;
                align-items: center;
                margin-bottom: 5px;
                user-select: none;
            `;
            
            const label = document.createElement('span');
            label.textContent = '📷 ' + cam.name;
            label.style.cssText = `
                color: #00d4ff;
                font-size: 12px;
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            `;
            
            const controls = document.createElement('div');
            controls.style.cssText = 'display: flex; gap: 5px;';
            
            const minimizeBtn = document.createElement('button');
            minimizeBtn.textContent = '−';
            minimizeBtn.style.cssText = `
                background: #333;
                color: #fff;
                border: none;
                width: 20px;
                height: 20px;
                border-radius: 3px;
                cursor: pointer;
                font-size: 14px;
                line-height: 1;
            `;
            minimizeBtn.onclick = (e) => {
                e.stopPropagation();
                const video = box.querySelector('video');
                const status = box.querySelector('.status-text');
                if (video.style.display === 'none') {
                    video.style.display = 'block';
                    if (status) status.style.display = 'block';
                    minimizeBtn.textContent = '−';
                } else {
                    video.style.display = 'none';
                    if (status) status.style.display = 'none';
                    minimizeBtn.textContent = '+';
                }
            };
            
            controls.appendChild(minimizeBtn);
            header.appendChild(label);
            header.appendChild(controls);
            
            const video = document.createElement('video');
            video.id = 'video-' + cam.id;
            video.autoplay = true;
            video.playsInline = true;
            video.muted = true;
            video.style.cssText = `
                width: 100%;
                height: auto;
                aspect-ratio: 4/3;
                background: #000;
                border-radius: 4px;
                display: block;
            `;
            
            const status = document.createElement('div');
            status.id = 'status-' + cam.id;
            status.className = 'status-text';
            status.textContent = 'Connecting...';
            status.style.cssText = `
                color: #ffab00;
                font-size: 10px;
                margin-top: 4px;
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            `;
            
            box.appendChild(header);
            box.appendChild(video);
            box.appendChild(status);
            container.appendChild(box);
            
            // Make draggable
            makeDraggable(box, header);
        });
        
        document.body.appendChild(container);
        
        // Connect to cameras
        cameras.forEach(cam => connectCamera(cam.id));
        
        // Load xterm and create terminal
        loadXterm(() => {
            createTerminal(container);
        });
    }
    
    function createTerminal(container) {
        const box = document.createElement('div');
        box.id = 'terminal-box';
        box.className = 'camera-box-draggable';
        box.style.cssText = `
            position: absolute;
            top: 500px;
            left: 20px;
            background: rgba(0, 0, 0, 0.95);
            border-radius: 8px;
            padding: 8px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.5);
            cursor: move;
            width: 700px;
            height: 350px;
            resize: both;
            overflow: hidden;
            border: 1px solid #00d4ff33;
        `;
        
        const header = document.createElement('div');
        header.className = 'drag-handle';
        header.style.cssText = `
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 5px;
            user-select: none;
        `;
        
        const label = document.createElement('span');
        label.textContent = '💻 Terminal';
        label.style.cssText = `
            color: #00d4ff;
            font-size: 12px;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
        `;
        
        const controls = document.createElement('div');
        controls.style.cssText = 'display: flex; gap: 5px;';
        
        const minimizeBtn = document.createElement('button');
        minimizeBtn.textContent = '−';
        minimizeBtn.style.cssText = `
            background: #333;
            color: #fff;
            border: none;
            width: 20px;
            height: 20px;
            border-radius: 3px;
            cursor: pointer;
            font-size: 14px;
            line-height: 1;
        `;
        
        const termContainer = document.createElement('div');
        termContainer.id = 'terminal-container';
        termContainer.style.cssText = `
            width: 100%;
            height: calc(100% - 25px);
        `;
        
        minimizeBtn.onclick = (e) => {
            e.stopPropagation();
            if (termContainer.style.display === 'none') {
                termContainer.style.display = 'block';
                minimizeBtn.textContent = '−';
                box.style.height = '350px';
            } else {
                termContainer.style.display = 'none';
                minimizeBtn.textContent = '+';
                box.style.height = 'auto';
            }
        };
        
        controls.appendChild(minimizeBtn);
        header.appendChild(label);
        header.appendChild(controls);
        box.appendChild(header);
        box.appendChild(termContainer);
        container.appendChild(box);
        
        makeDraggable(box, header);
        
        // Initialize xterm
        setTimeout(() => {
            initTerminal(termContainer);
        }, 100);
        
        // Handle resize
        const resizeObserver = new ResizeObserver(() => {
            if (window.term && window.fitAddon) {
                window.fitAddon.fit();
                sendResize();
            }
        });
        resizeObserver.observe(box);
    }
    
    function initTerminal(container) {
        const term = new Terminal({
            cursorBlink: true,
            fontSize: 13,
            fontFamily: 'Menlo, Monaco, "Courier New", monospace',
            theme: {
                background: '#1a1a2e',
                foreground: '#eee',
                cursor: '#00d4ff',
                selection: '#00d4ff44',
            },
            allowTransparency: true,
        });
        
        const fitAddon = new FitAddon.FitAddon();
        term.loadAddon(fitAddon);
        
        term.open(container);
        fitAddon.fit();
        
        window.term = term;
        window.fitAddon = fitAddon;
        
        // Connect WebSocket
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws = new WebSocket(`${wsProtocol}//${window.location.host}/live/terminal`);
        window.termWs = ws;
        
        ws.onopen = () => {
            term.writeln('\x1b[32mConnected to terminal\x1b[0m');
            sendResize();
        };
        
        ws.onmessage = (event) => {
            term.write(event.data);
        };
        
        ws.onclose = () => {
            term.writeln('\x1b[31mDisconnected from terminal\x1b[0m');
        };
        
        ws.onerror = (e) => {
            term.writeln('\x1b[31mTerminal connection error\x1b[0m');
        };
        
        term.onData((data) => {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: 'input', data: data }));
            }
        });
    }
    
    function sendResize() {
        if (window.term && window.termWs && window.termWs.readyState === WebSocket.OPEN) {
            window.termWs.send(JSON.stringify({
                type: 'resize',
                rows: window.term.rows,
                cols: window.term.cols
            }));
        }
    }
    
    function makeDraggable(element, handle) {
        let pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
        
        handle.onmousedown = dragMouseDown;
        
        function dragMouseDown(e) {
            e.preventDefault();
            pos3 = e.clientX;
            pos4 = e.clientY;
            document.onmouseup = closeDragElement;
            document.onmousemove = elementDrag;
        }
        
        function elementDrag(e) {
            e.preventDefault();
            pos1 = pos3 - e.clientX;
            pos2 = pos4 - e.clientY;
            pos3 = e.clientX;
            pos4 = e.clientY;
            element.style.top = (element.offsetTop - pos2) + "px";
            element.style.left = (element.offsetLeft - pos1) + "px";
        }
        
        function closeDragElement() {
            document.onmouseup = null;
            document.onmousemove = null;
        }
    }
    
    const pcs = {};
    
    async function connectCamera(camId) {
        const video = document.getElementById('video-' + camId);
        const status = document.getElementById('status-' + camId);
        
        if (!video || !status) return;
        
        if (pcs[camId]) { pcs[camId].close(); pcs[camId] = null; }
        
        status.textContent = 'Connecting...';
        status.style.color = '#ffab00';
        
        try {
            const pc = new RTCPeerConnection({
                iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
            });
            pcs[camId] = pc;
            
            pc.ontrack = (event) => {
                video.srcObject = event.streams[0];
                status.textContent = 'Connected';
                status.style.color = '#00c853';
            };
            
            pc.oniceconnectionstatechange = () => {
                if (pc.iceConnectionState === 'failed' || pc.iceConnectionState === 'disconnected') {
                    status.textContent = 'Reconnecting...';
                    status.style.color = '#ff1744';
                    setTimeout(() => connectCamera(camId), 3000);
                }
            };
            
            pc.addTransceiver('video', { direction: 'recvonly' });
            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            
            await new Promise(r => {
                if (pc.iceGatheringState === 'complete') r();
                else { 
                    pc.onicegatheringstatechange = () => { 
                        if (pc.iceGatheringState === 'complete') r(); 
                    }; 
                    setTimeout(r, 3000); 
                }
            });
            
            const response = await fetch('/live/offer/' + camId, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type })
            });
            
            const answer = await response.json();
            await pc.setRemoteDescription(new RTCSessionDescription(answer));
            
        } catch (e) {
            console.error('Camera connection error:', e);
            status.textContent = 'Error - Retrying...';
            status.style.color = '#ff1744';
            setTimeout(() => connectCamera(camId), 3000);
        }
    }
    
    // Start injection
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', injectCameraFeed);
    } else {
        injectCameraFeed();
    }
})();

