Armor MVP (OpenCV)

Minimal runnable implementation for detecting and stabilizing armor plates in video/camera streams using traditional vision, as specified in docs/docs.md.

Build

- Requirements: OpenCV (core, imgproc, highgui, videoio, video)
- Commands:
  - mkdir -p build && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  - cmake --build build -j

Run

- Camera (default device 0):
  - ./build/armor_mvp
- With config:
  - ./build/armor_mvp --config config/app.yaml
- Override with video file:
  - ./build/armor_mvp --config config/app.yaml --video /path/to/video.mp4

Config (config/app.yaml)

See inline defaults; keys match docs:

input:
  device_id: 0
  video: ""
  width: 1280
  height: 720
  fps: 60

bin:
  method: "adaptive"    # adaptive|otsu|auto
  block_size: 21
  C: 5

morph:
  ksize: 3

filter:
  ratio_min: 2.0
  ratio_max: 3.2
  area_min_px: 800
  rectangularity_min: 0.75

detector:
  max_candidates: 6

tracker:
  iou_thresh: 0.2
  max_missed: 8
  smooth_alpha: 0.25

ui:
  show_fps: true
  window: "Armor MVP"
  save_video: ""

Notes

- Press 'q' or ESC to exit.
- For best performance, use Release build.
- If you don't have a camera, override with a video file.

