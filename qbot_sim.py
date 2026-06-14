import asyncio
import websockets
import json
import threading
import time

class QBot2Sim:
    def __init__(self, host="ws://127.0.0.1:8000/api/ws/sim"):
        self.uri = host
        self.position = [0.0, 0.0, 0.0]
        self.heading = 0.0
        self.lidar_front = 20.0
        self._connected = False
        
        # Dedicated thread for persistent WebSocket connection (Two-way telemetry)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_loop, daemon=True)
        self._thread.start()
        
        # Wait for connection to establish
        timeout = 5
        start = time.time()
        while not self._connected and time.time() - start < timeout:
            time.sleep(0.1)
            
        if not self._connected:
            print("Warning: Could not connect to QUANSER QBot 2 simulation backend.")

    def _start_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ws_loop())

    async def _ws_loop(self):
        while True:
            try:
                async with websockets.connect(self.uri) as ws:
                    self._ws = ws
                    self._connected = True
                    # Continuous receive loop
                    async for message in ws:
                        try:
                            data = json.loads(message)
                            if "telemetry" in data:
                                t = data["telemetry"]
                                self.position = [t.get("x", 0.0), 0.0, t.get("z", 0.0)]
                                self.heading = t.get("heading", 0.0)
                                self.lidar_front = t.get("lidar", 20.0)
                        except:
                            pass
            except Exception as e:
                self._connected = False
                await asyncio.sleep(1)

    def _send(self, cmd):
        if self._connected and hasattr(self, '_ws'):
            async def send_msg():
                try:
                    await self._ws.send(json.dumps(cmd))
                except:
                    pass
            asyncio.run_coroutine_threadsafe(send_msg(), self._loop)

    def drive_forward(self):
        self._send({"action": "walk"})
        
    def stop(self):
        self._send({"action": "stand"})
        
    def reset(self):
        self._send({"action": "reset"})
        
    def turn_left(self):
        self._send({"action": "turn_left"})
        
    def turn_right(self):
        self._send({"action": "turn_right"})

    def move_to(self, x, z):
        """Commands the robot engine to orient and drive to a specific world coordinate."""
        self._send({"action": "move_to", "target": [x, z]})

    def start_auto_patrol(self):
        self._send({"action": "auto_patrol"})
        
    def log(self, message):
        print(message)
        self._send({"log": message})

    def get_position(self):
        """Returns the real-time position [x, y, z] streamed from the simulation."""
        return self.position

    def get_lidar_front(self):
        """Returns the real-time forward LiDAR depth value streamed from the simulation."""
        return self.lidar_front
