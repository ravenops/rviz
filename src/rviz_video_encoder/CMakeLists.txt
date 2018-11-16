add_library(rviz_video_encoder
  video_encoder.c
)

target_link_libraries(rviz_video_encoder
  "swscale"
  "avcodec"
  "avutil"
  "avformat"
  "xxhash"
)

set_target_properties(rviz_video_encoder
                      PROPERTIES OUTPUT_NAME rviz_video_encoder
                      PREFIX ""
                      )


install(TARGETS rviz_video_encoder
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
