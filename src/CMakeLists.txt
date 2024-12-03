set(publicHeaderDir "${CMAKE_SOURCE_DIR}/include")
set(publicHeaders
        "RoverReceiver.hpp"
        "LidarReceiver.hpp"
        "CameraReceiver.hpp"
        "KeyboardController.hpp"
        "AutonomousController.hpp"
)

set(publicHeadersFull)
foreach(header IN LISTS publicHeader)
    list(APPEND publicHeadersFull "${publicHeaderDir}/${header}")
endforeach ()

set(privateHeaders
        "TCPStreamer.hpp"
        "KeyListener.hpp"
        "CSVWriter.hpp"
        "CommandStreamer.hpp"
        "CommandsEnum.hpp"
        "KeysEnum.hpp"
        "TCPConnector.hpp"
)
set(privateHeadersFull)
foreach(header IN LISTS privateHeaders)
    list(APPEND privateHeadersFull "${publicHeaderDir}/${header}")
endforeach ()
set(sources
        "RoverReceiver.cpp"
        "LidarReceiver.cpp"
        "CameraReceiver.cpp"
        "KeyListener.cpp"
        "TCPStreamer.cpp"
        "CommandStreamer.cpp"
        "KeyboardController.cpp"
        "AutonomousController.cpp"
        "TCPConnector.cpp"
)

add_library(lib ${sources} ${privateHeadersFull} ${publicHeadersFull})
target_include_directories(lib PUBLIC "${CMAKE_SOURCE_DIR}/include")

include(FetchContent)

set(SIMPLE_SOCKET_BUILD_TESTS_OFF)
FetchContent_Declare(
        SimpleSocket
        GIT_REPOSITORY https://github.com/markaren/SimpleSocket.git
        GIT_TAG e0e989bdd52137d743a4599e25167d5681a0de00
)
FetchContent_MakeAvailable(SimpleSocket)

set(OpenCV_DIR $ENV{OpenCV_DIR})
set(OpenCV)
find_package(OpenCV CONFIG REQUIRED)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 251d2723315a1ca3952c4d074a3bd038222f4289
)
FetchContent_MakeAvailable(threepp)


target_link_libraries(lib PUBLIC simple_socket threepp::threepp ${OpenCV_LIBRARIES})