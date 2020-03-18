add_library(
    myvo
    camera.cpp
)

target_link_libraries(
    myvo 
    ${Sophus_LIBRARIES} 
    ${OpenCV_LIBS}
    g2o_core g2o_stuff
)