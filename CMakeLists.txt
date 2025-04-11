# CMakeList.txt : KalmanFilterTest の CMake プロジェクト。ソースを含めて、次を定義します:
# プロジェクト専用ロジックはこちらです。
#
cmake_minimum_required (VERSION 3.8)

project ("KalmanFilterTest")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# ソースをこのプロジェクトの実行可能ファイルに追加します。
add_executable (KalmanFilterTest "KalmanFilterTest.cpp" "KalmanFilterTest.h" "KalmanFilter.cpp" "KalmanFilter.h")

# TODO: テストを追加し、必要な場合は、ターゲットをインストールします。
#set(CMAKE_CXX_FLAGS "-H")
#echo %INCLUDE%
#target_link_libraries (example Eigen3::Eigen)
target_link_libraries(mylibrary Eigen3::Eigen)
