from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import os
import uvicorn

app = FastAPI()

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, 'static')
os.makedirs(STATIC_DIR, exist_ok=True)

app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

# Store connected clients to broadcast
connected_clients = []

@app.get("/", response_class=HTMLResponse)
async def get_index():
    with open(os.path.join(STATIC_DIR, "index.html"), "r", encoding="utf-8") as f:
        return f.read()

@app.websocket("/api/ws/sim")
async def websocket_sim(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    try:
        while True:
            # Receive data from Python script
            data = await websocket.receive_text()
            # Broadcast to all connected clients (the browser UI)
            for client in connected_clients:
                if client != websocket:
                    try:
                        await client.send_text(data)
                    except:
                        pass
    except Exception as e:
        pass
    finally:
        connected_clients.remove(websocket)

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
