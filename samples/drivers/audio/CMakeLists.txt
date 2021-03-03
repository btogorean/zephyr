# Find Zephyr. This also loads Zephyr's build system.
cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr)
project(AUDIO)

# Add your source file to the "app" target. This must come after
# find_package(Zephyr) which defines the target.
target_sources(app PRIVATE src/main.c)

target_link_libraries(app PUBLIC -lm)
target_link_libraries(app PUBLIC -lc)
target_link_libraries(app PUBLIC -Wl,-u,__extendsfdf2)
target_link_libraries(app PUBLIC -Wl,-u,__truncdfsf2)
target_link_libraries(app PUBLIC -Wl,-u,__divsf3)
target_link_libraries(app PUBLIC -Wl,-u,__divdf3)
target_link_libraries(app PUBLIC -Wl,-u,__adddf3)
target_link_libraries(app PUBLIC -Wl,-u,__subdf3)
target_link_libraries(app PUBLIC -Wl,-u,__ieee754_sqrtf)
target_link_libraries(app PUBLIC -Wl,-u,__ieee754_remainderf)
target_link_libraries(app PUBLIC -Wl,-u,__nedf2)
