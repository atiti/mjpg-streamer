
MJPG_STREAMER_PLUGIN_OPTION(input_fbimx "Framebuffer capture and jpeg encode for i.MX6")

include_directories(/workspace/libimxvpuapi/)
link_directories(/workspace/libimxvpuapi/build/)
MJPG_STREAMER_PLUGIN_COMPILE(input_fbimx input_fbimx.c ipu_csc.c)

target_link_libraries(input_fbimx imxvpuapi)

