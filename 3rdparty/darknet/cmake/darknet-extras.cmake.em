if (_CMAKE_DARKNET_EXTRAS_INCLUDED)
   return()
endif()
set(_CMAKE_DARKNET_EXTRAS_INCLUDED)

function(add_darknet_definitions target)
  find_package(CUDA)
  if(CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})
    target_compile_definitions(${target} PUBLIC -DGPU)
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
      include_directories(${CUDNN_INCLUDE_DIR})
      target_compile_definitions(${target} PUBLIC -DCUDNN)
    endif()
  endif()
  find_package(OpenCV)
  if(OpenCV_FOUND)
    if ("${OpenCV_VERSION}" VERSION_LESS "3.0.0")
      include_directories(${OpenCV_INCLUDE_DIRS})
      target_compile_definitions(${target} PUBLIC -DOPENCV)
    endif()
  endif()
endfunction()
