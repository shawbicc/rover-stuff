import asyncio
import websockets
import cv2
import base64
import numpy as np
import config
from concurrent.futures import ThreadPoolExecutor

async def server_handler(websocket, path, camera_index):
    cap = cv2.VideoCapture(camera_index)

    while cap.isOpened():
        try:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame = cv2.resize(frame, (400, 300))
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            frame_data = base64.b64encode(buffer).decode('utf-8')

            await websocket.send(frame_data)

        except websockets.exceptions.ConnectionClosed:
            print(f"Connection closed for camera {camera_index}.")
            break

    cap.release()

def run_server(camera_index, port):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    start_server = websockets.serve(
        lambda websocket, path, ci=camera_index: server_handler(websocket, path, ci),
        config.IP, port
    )

    loop.run_until_complete(start_server)
    loop.run_forever()

async def main():
    with ThreadPoolExecutor() as executor:
        futures = [executor.submit(run_server, camera_index, port) for camera_index, port in zip(config.CAMERA_INDICES, config.PORTS)]
        await asyncio.gather(*futures)

if __name__ == "__main__":
    asyncio.run(main())
