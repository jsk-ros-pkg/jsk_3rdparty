# This is CMakeLists.txt for darknet
cmake_minimum_required(VERSION 2.8)
project(darknet C CXX)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -falign-functions=4")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

## Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--export-dynamic -Wall -Wno-sign-compare -fPIC")
endif()

#thread
find_package(Threads)
list(APPEND LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

# CUDA
find_package(CUDA)
if (CUDA_FOUND)
  message(STATUS "CUDA Version: " ${CUDA_VERSION_STRINGS})
  message(STATUS "CUDA Libararies: " ${CUDA_LIBRARIES})
  include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} ${CUDA_cusparse_LIBRARY})
  list(APPEND CUDA_NVCC_FLAGS "-arch=sm_35;-O2;-Xcompiler;-fPIC;")
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  add_definitions(-DGPU)
  cuda_include_directories(src)

  # cuDNN
  find_path(CUDNN_INCLUDE_DIR
    NAMES cudnn.h
    PATHS ${CUDA_INCLUDE_DIRS} /usr/local/cuda/include
    PATH_SUFFIXES include)
  get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
  find_library(CUDNN_LIBRARY
    NAMES cudnn
    PATHS ${CUDNN_LIB_DIR} /usr/local/cuda/lib64)
  if (EXISTS ${CUDNN_INCLUDE_DIR} AND EXISTS ${CUDNN_LIBRARY})
    message(STATUS "Found cuDNN: " ${CUDNN_LIBRARY})
    list(APPEND LIBRARIES ${CUDNN_LIBRARY})
    include_directories(${CUDNN_INCLUDE_DIR})
    add_definitions(-DCUDNN)
  endif()

else()
  list(APPEND LIBRARIES "m")
endif()

#BOOST
find_package(Boost REQUIRED python)
find_package(PythonLibs REQUIRED)

# C API is deprecated from 3.0 and build fails on errors
# ref: https://github.com/opencv/opencv/issues/6585
find_package(OpenCV)
if(OpenCV_FOUND)
  message(STATUS "Found OpenCV ${OpenCV_VERSION}: ${OpenCV_LIBRARIES}")
  if ("${OpenCV_VERSION}" VERSION_LESS "3.0.0")
    add_definitions(-DOPENCV)
  else ()
    message(WARNING "OpenCV >= 3 does not support C API. Disabled OpenCV Feature")
  endif()
endif()

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIR})

include_directories(src)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.c)
list(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/server.c")
list(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/old.c")
list(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/cpu_gemm.c")
list(REMOVE_ITEM SRC_FILES "${PROJECT_SOURCE_DIR}/src/darknet.c")

if (CUDA_FOUND)
  file(GLOB CU_SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cu)
  list(REMOVE_ITEM CU_SRC_FILES "${PROJECT_SOURCE_DIR}/src/yolo_kernels.cu")
  cuda_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
  set(CUDA_ARCH_BIN "20 30 32 35 37 50 52" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
  set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

  #compute flags macros
  macro(cuda_compute_target_flags arch_bin arch_ptx cuda_nvcc_target_flags)
    string(REGEX REPLACE "\\." "" ARCH_BIN_WITHOUT_DOTS "${${arch_bin}}")
    string(REGEX REPLACE "\\." "" ARCH_PTX_WITHOUT_DOTS "${${arch_ptx}}")
    set(cuda_computer_target_flags_temp "")

    # Tell NVCC to add binaries for the specified GPUs
    string(REGEX MATCHALL "[0-9()]+" ARCH_LIST "${ARCH_BIN_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)
      if(ARCH MATCHES "([0-9]+)\\(([0-9]+)\\)")
        # User explicitly specified PTX for the concrete BIN
        set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${CMAKE_MATCH_2},code=sm_${CMAKE_MATCH_1})
      else()
        # User didn't explicitly specify PTX for the concrete BIN, we assume PTX=BIN
        set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=sm_${ARCH})
      endif()
    endforeach()

    # Tell NVCC to add PTX intermediate code for the specified architectures
    string(REGEX MATCHALL "[0-9]+" ARCH_LIST "${ARCH_PTX_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)
      set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=compute_${ARCH})
    endforeach()
    set(${cuda_nvcc_target_flags} ${cuda_computer_target_flags_temp})
  endmacro()

  macro(append_target_arch_flags)
    set(cuda_nvcc_target_flags "")
    cuda_compute_target_flags(CUDA_ARCH_BIN CUDA_ARCH_PTX cuda_nvcc_target_flags)
    if (cuda_nvcc_target_flags)
      message(STATUS "CUDA NVCC target flags: ${cuda_nvcc_target_flags}")
      list(APPEND CUDA_NVCC_FLAGS ${cuda_nvcc_target_flags})
    endif()
  endmacro()

  append_target_arch_flags()

  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

  cuda_compile(cuda_objs ${CU_SRC_FILES})
endif()

# build
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${CU_SRC_FILES} ${cuda_objs})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES} ${OpenCV_LIBS})
add_executable(${PROJECT_NAME}_bin src/darknet.c)
target_link_libraries(${PROJECT_NAME}_bin ${PROJECT_NAME} ${LIBRARIES} ${OpenCV_LIBS})
set_target_properties(${PROJECT_NAME}_bin
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# install
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_bin
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/cfg/
  DESTINATION cfg)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/data/
  DESTINATION data)
