import asyncio
import cv2
import websockets

CAMERA = "/dev/video2"

HOST = "0.0.0.0"
PORT = 8765


async def stream(websocket):
    cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"Cannot open camera: {CAMERA}")
        return

    # Configure camera
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

    print("Camera configuration:")
    print("  FOURCC:", int(cap.get(cv2.CAP_PROP_FOURCC)))
    print("  Width :", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("  Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print("Client connected")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            # Ensure output is always 640x480
            frame = cv2.resize(frame, (1080, 675))

            ok, jpeg = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    80,
                ],
            )

            if not ok:
                continue

            await websocket.send(jpeg.tobytes())

            # Give control back to asyncio
            await asyncio.sleep(0)

    except websockets.ConnectionClosed:
        print("Client disconnected")

    finally:
        cap.release()


async def main():
    async with websockets.serve(
        stream,
        HOST,
        PORT,
        max_size=None,
    ):
        print(f"Listening on ws://{HOST}:{PORT}")
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
