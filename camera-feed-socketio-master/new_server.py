import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

import socketio
import asyncio
import uvicorn
import time
import cv2
import camera_config

# Create a Socket.IO server with CORS allowed origins
sio = socketio.AsyncServer(
    logger=True,
    async_mode="asgi",
    cors_allowed_origins="*"
)
app = socketio.ASGIApp(sio)

@sio.on(event="join_feed", namespace="/feed")
async def join_feed(sid, index: int):
    await sio.enter_room(sid, f"feed_receiver_{index}", namespace="/feed")

@sio.on(event="leave_feed", namespace="/feed")
async def leave_feed(sid, index):
    await sio.leave_room(sid, f"feed_receiver_{index}", namespace="/feed")

cams = set()
camera_range = range(0, 3)

@sio.event(namespace="/feed")
async def connect(sid, environ, auth):
    await sio.emit("camera_change", namespace="/feed", data=list(cams), skip_sid=True)

@sio.event(namespace="/feed")
def disconnect(sid):
    pass

def get_transmit_permission(camera_index):
    if "/feed" not in sio.manager.rooms:
        return False

    feed_rooms = sio.manager.rooms["/feed"]
    if f"feed_receiver_{camera_index}" not in feed_rooms:
        return False

    return True

async def background_task(camera_index: int):
    print("background_task", camera_index)
    cap = cv2.VideoCapture(camera_index)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        if not get_transmit_permission(camera_index):
            await asyncio.sleep(0.0001)
            continue

        frame = cv2.resize(frame, (400, 300))
        sus, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not sus:
            continue

        start_time = time.time()
        await sio.emit(
            f"send_feed_{camera_index}",
            namespace="/feed",
            room=f"feed_receiver_{camera_index}",
            data=(buffer.tobytes(), start_time),
        )
        await asyncio.sleep(0.0001)

    print(f"camera_index: {camera_index} released")
    if camera_index in cams:
        cams.remove(camera_index)
    cap.release()

async def look_for_camera_index():
    loop = asyncio.get_event_loop()
    asyncio.set_event_loop(loop)

    while True:
        indices = []
        for idx in camera_range:
            if idx not in list(cams):
                capture = cv2.VideoCapture(idx)
                if capture.isOpened():
                    indices.append(idx)
                capture.release()
            await asyncio.sleep(0.01)

        new_cams = set(indices)
        diff = new_cams.difference(cams)
        if len(diff) > 0:
            for new_cam in diff:
                loop.create_task(background_task(new_cam))
            cams.update(diff)
            await sio.emit(
                "camera_change",
                namespace="/feed",
                data=list(cams),
                skip_sid=True,
            )

        await asyncio.sleep(2)

async def run_server():
    config = uvicorn.Config(
        app,
        host="0.0.0.0",
        port=5000,
        log_level="info",
        loop="asyncio",
        workers=5,
        lifespan="on",
    )
    server = uvicorn.Server(config)
    await server.serve()

async def main():
    await asyncio.gather(
        run_server(),
        look_for_camera_index(),
    )

if __name__ == "__main__":
    asyncio.run(main())
