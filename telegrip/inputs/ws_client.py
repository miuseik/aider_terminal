"""
WebSocket client for connecting to Aider Server.
Handles bidirectional message forwarding between VR clients and server.
"""

import asyncio
import json
import ssl
import websockets
import logging
from typing import Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class VRWebSocketClient:
    """WebSocket client that connects to Aider Server."""
    
    def __init__(self, config, vr_handler):
        self.config = config
        self.vr_handler = vr_handler
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.is_connected = False
        self.client_id = "terminal"  # Terminal always uses this ID
        
        # SSL context
        self.ssl_context = None
    
    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket client connection."""
        # Client only needs to trust server cert, no client cert needed
        ssl_context = ssl.create_default_context()
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE
        
        return ssl_context
    
    async def connect(self):
        """Connect to Aider Server."""
        self.ssl_context = self.setup_ssl()
        if self.ssl_context is None:
            logger.error("Failed to setup SSL")
            return False
        
        host = self.config.server_host
        port = self.config.websocket_port
        ws_url = f"wss://{host}:{port}/vr/terminal"
        print(f"ws_url: {ws_url}")
        try:
            logger.info(f"🔌 Connecting to Aider Server: {ws_url}")
            self.websocket = await websockets.connect(
                ws_url,
                ssl=self.ssl_context
            )
            self.is_connected = True
            print(f"✅ Connected to Aider Server")
            await self.on_connected()
            
            # Start message receiving task
            asyncio.create_task(self.receive_messages())
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to Aider Server: {e}")
            # Auto reconnect after 3 seconds
            logger.info(f"🔄 Reconnecting in 3 seconds...")
            await asyncio.sleep(3)
            return await self.connect()
    
    async def on_connected(self):
        """Called when connection is successfully established."""
        print(f"✅ Successfully connected to Aider Server at {self.config.server_host}:{self.config.websocket_port}")
    
    async def disconnect(self):
        """Disconnect from Aider Server."""
        self.is_connected = False
        if self.websocket:
            try:
                await self.websocket.close()
            except Exception:
                pass
            logger.info("🔴 Disconnected from Aider Server")
    
    async def receive_messages(self):
        """Receive messages from server and forward to VR handler."""
        try:
            async for message in self.websocket:
                if not self.is_connected:
                    break
                
                try:
                    data = json.loads(message)
                    
                    # Forward to VR handler for processing
                    await self.vr_handler.process_message(message)
                    
                except json.JSONDecodeError:
                    logger.warning(f"⚠️ Received non-JSON message")
                except Exception as e:
                    logger.error(f"❌ Error processing message: {e}")
        
        except websockets.exceptions.ConnectionClosedOK:
            logger.info("❌ Connection closed (normal)")
            self.is_connected = False
            # Auto reconnect
            logger.info(f"🔄 Reconnecting in 3 seconds...")
            await asyncio.sleep(3)
            await self.connect()
        except websockets.exceptions.ConnectionClosedError as e:
            logger.error(f"❌ Connection closed (error): {e}")
            self.is_connected = False
            # Auto reconnect
            logger.info(f"🔄 Reconnecting in 3 seconds...")
            await asyncio.sleep(3)
            await self.connect()
        except Exception as e:
            logger.error(f"❌ Receive error: {e}")
            self.is_connected = False
            # Auto reconnect
            logger.info(f"🔄 Reconnecting in 3 seconds...")
            await asyncio.sleep(3)
            await self.connect()
    
    async def send_message(self, data: dict):
        """Send message to Aider Server."""
        if not self.is_connected or not self.websocket:
            logger.warning("⚠️ Not connected to server, cannot send message")
            return False
        
        try:
            await self.websocket.send(json.dumps(data))
            return True
        except Exception as e:
            logger.error(f"❌ Failed to send message: {e}")
            return False
    
    async def send_vr_data(self, vr_data: dict):
        """Send VR controller data to server."""
        return await self.send_message(vr_data)
    
    async def send_command(self, action: str, **kwargs):
        """Send command to server."""
        command = {"action": action, **kwargs}
        return await self.send_message(command)
