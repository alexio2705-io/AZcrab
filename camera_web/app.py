#!/usr/bin/env python3
import io
from threading import Condition

from flask import Flask, Response, render_template

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

FRAME_SIZE = (1280, 720)
HTTP_PORT = 8000


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


def start_camera(camera_num):
    try:
        picam2 = Picamera2(camera_num=camera_num)
        config = picam2.create_video_configuration(main={"size": FRAME_SIZE})
        picam2.configure(config)
        output = StreamingOutput()
        picam2.start_recording(MJPEGEncoder(), FileOutput(output))
        return picam2, output
    except Exception as exc:
        print(f"Камера {camera_num} недоступна: {exc}")
        return None, None


app = Flask(__name__)

cameras = {cam_id: start_camera(cam_id) for cam_id in (0, 1)}

PLACEHOLDER_SVG = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">'
    '<rect width="100%" height="100%" fill="#1B1E23"/>'
    '<text x="50%" y="50%" fill="#7E838C" font-size="24" font-family="ui-monospace,Menlo,monospace" '
    'text-anchor="middle" dominant-baseline="middle">Камера {cam_id} недоступна</text>'
    '</svg>'
)


def mjpeg_generator(output):
    while True:
        with output.condition:
            output.condition.wait()
            frame = output.frame
        yield (b"--FRAME\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/stream/<int:cam_id>.mjpg")
def stream(cam_id):
    entry = cameras.get(cam_id)
    if not entry or entry[1] is None:
        svg = PLACEHOLDER_SVG.format(w=FRAME_SIZE[0], h=FRAME_SIZE[1], cam_id=cam_id)
        return Response(svg, mimetype="image/svg+xml")
    _, output = entry
    return Response(
        mjpeg_generator(output),
        mimetype="multipart/x-mixed-replace; boundary=FRAME",
    )


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
    finally:
        for picam2, _ in cameras.values():
            if picam2:
                picam2.stop_recording()
