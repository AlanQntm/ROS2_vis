// Entry point
#include "armor/app.hpp"
#include <iostream>

int main(int argc, char** argv) {
  std::string config_path = "config/app.yaml";
  std::string video_override;
  std::string save_override;
  std::string dump_path;
  bool no_window = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      config_path = argv[++i];
    } else if ((arg == "-v" || arg == "--video") && i + 1 < argc) {
      video_override = argv[++i];
    } else if ((arg == "-s" || arg == "--save") && i + 1 < argc) {
      save_override = argv[++i];
    } else if (arg == "--no-window") {
      no_window = true;
    } else if ((arg == "-d" || arg == "--dump") && i + 1 < argc) {
      dump_path = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: armor_mvp [-c config.yaml] [-v video.mp4] [--save out.mp4] [--no-window] [--dump results.jsonl]\n";
      return 0;
    }
  }

  armor::App app;
  app.load_config(config_path);
  if (!video_override.empty()) app.override_video(video_override);
  if (no_window) app.set_headless(true);
  if (!save_override.empty()) app.set_save_video(save_override);
  if (!dump_path.empty()) app.set_dump_path(dump_path);
  return app.run();
}
