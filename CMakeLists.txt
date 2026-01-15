cmake_minimum_required(VERSION 3.10...3.31)
# 启用 C 和 CXX 语言（Linux 上 VTK/MPI 需要 C 语言支持）
project(LingerPerceiver VERSION 0.0.1 LANGUAGES C CXX)
 
 # 如果使用 MSVC 编译器，向编译器请求 UTF-8 输出/诊断支持
 if (MSVC)
	 message(STATUS "MSVC detected: adding /utf-8 compile option to request UTF-8 diagnostics")
	 add_compile_options(/utf-8)
 endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(USE_PCL "Use PCL libraries" ON)
option(USE_LIVOX "Use Livox SDK2" ON)
option(BUILD_GUI "Build GUI application (Qt/VTK)" ON)
option(BUILD_HEADLESS "Build headless service without Qt/VTK" OFF)
option(ENABLE_TRANSLATIONS "Enable Qt translations (i18n)" ON)
option(STATIC_LINK_STDLIB "Statically link libstdc++ and libgcc (Linux)" OFF)

# GUI 构建依赖 Qt/VTK；Headless 不需要。默认开启 GUI 构建。
set(USE_QT ${BUILD_GUI})

# 如果未定义 CMAKE_PREFIX_PATH，则以当前（可能为空）的值初始化（保留现有前缀路径）。
if(NOT DEFINED CMAKE_PREFIX_PATH)
 set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
endif()

# 检查本地的 3rdParty 依赖目录
set(LOCAL_3RDPARTY_DIR "${CMAKE_SOURCE_DIR}/3rdParty")

# 定义依赖库版本，方便统一管理
set(PCL_VERSION_DIR "PCL-1.15.1")
set(VTK_VERSION_DIR "vtk-9.4")

# 设置 Qt 路径（CACHE 变量允许在命令行被覆盖，例如 -DQT_DIR="C:/Qt/..."）
# 如果不想设置环境变量，可以在这里指定默认路径
set(QT_DIR "D:/Qt/Qt5.12.7/5.12.7/msvc2017_64" CACHE PATH "Path to Qt installation")
list(APPEND CMAKE_PREFIX_PATH "${QT_DIR}")

# PCL 与 VTK 的查找与配置
if(EXISTS "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK")
 list(APPEND CMAKE_PREFIX_PATH "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK")
 if(EXISTS "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK/include/${VTK_VERSION_DIR}")
 include_directories("${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK/include/${VTK_VERSION_DIR}")
 endif()
endif()

set(EXTRA_COMPILE_DEFS "")

if(USE_QT)
 find_package(Qt5 COMPONENTS Widgets REQUIRED)
 message(STATUS "Qt5 Widgets found: ${Qt5Widgets_VERSION}")
 list(APPEND EXTRA_LIBS Qt5::Widgets)
 
 # 查找 Qt5 LinguistTools 用于翻译支持
 if(ENABLE_TRANSLATIONS)
  find_package(Qt5 COMPONENTS LinguistTools QUIET)
  if(Qt5LinguistTools_FOUND)
   message(STATUS "Qt5 LinguistTools found - translations enabled")
  else()
   message(WARNING "Qt5 LinguistTools not found - translations disabled")
   set(ENABLE_TRANSLATIONS OFF)
  endif()
 endif()
 
 if(NOT DEFINED VTK_DIR)
 if(EXISTS "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK/lib/cmake/${VTK_VERSION_DIR}")
 set(VTK_DIR "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK/lib/cmake/${VTK_VERSION_DIR}")
 elseif(EXISTS "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK/lib")
 set(VTK_DIR "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/VTK/lib")
 endif()
 endif()

 find_package(VTK REQUIRED COMPONENTS GUISupportQt RenderingOpenGL2)
 message(STATUS "VTK found: ${VTK_VERSION}")
 list(APPEND EXTRA_COMPILE_DEFS HAVE_QVTK)
 if(DEFINED VTK_LIBRARIES)
 list(APPEND EXTRA_LIBS ${VTK_LIBRARIES})
 endif()
endif()

if(USE_PCL)
 # Linux 平台：优先使用系统安装的 PCL
 if(UNIX AND NOT APPLE)
   # Headless 模式只需要核心组件，避免 VTK/可视化依赖
   if(BUILD_HEADLESS)
     find_package(PCL 1.14 REQUIRED COMPONENTS common io filters kdtree search octree sample_consensus segmentation features)
   else()
     find_package(PCL 1.14 REQUIRED)  # Debian trixie 的 PCL 版本
   endif()
 else()
   # Windows: 使用本地 3rdParty 目录
   if(NOT DEFINED PCL_DIR)
     set(PCL_DIR "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}")
   endif()
   find_package(PCL 1.15 REQUIRED HINTS ${PCL_DIR})
 endif()
 message(STATUS "PCL found: ${PCL_VERSION}")
 include_directories(${PCL_INCLUDE_DIRS})
 list(APPEND EXTRA_LIBS ${PCL_LIBRARIES})
endif()

if(USE_LIVOX)
 if(NOT DEFINED LIVOX_SDK2_ROOT)
 set(LIVOX_SDK2_ROOT "${LOCAL_3RDPARTY_DIR}/Livox-SDK2")
 endif()
 
 # Linux 平台：优先使用系统安装的 Livox SDK2
 if(UNIX AND NOT APPLE)
   find_library(LIVOX_SDK2_SYSTEM_LIB NAMES livox_lidar_sdk_static HINTS /usr/local/lib /usr/lib)
   if(LIVOX_SDK2_SYSTEM_LIB)
     message(STATUS "Livox SDK2 found in system: ${LIVOX_SDK2_SYSTEM_LIB}")
     set(LIVOX_SDK2_INCLUDE_DIR "/usr/local/include")
     set(LIVOX_SDK2_STATIC_LIB_RELEASE_PATH "${LIVOX_SDK2_SYSTEM_LIB}")
     set(LIVOX_SDK2_STATIC_LIB_DEBUG_PATH "${LIVOX_SDK2_SYSTEM_LIB}")
   elseif(EXISTS "${LIVOX_SDK2_ROOT}")
     # 回退到本地目录
     find_library(LIVOX_SDK2_STATIC_LIB_RELEASE NAMES livox_lidar_sdk_static HINTS ${LIVOX_SDK2_ROOT}/lib)
     if(LIVOX_SDK2_STATIC_LIB_RELEASE)
       message(STATUS "Livox SDK2 found locally: ${LIVOX_SDK2_STATIC_LIB_RELEASE}")
       set(LIVOX_SDK2_INCLUDE_DIR "${LIVOX_SDK2_ROOT}/include")
       set(LIVOX_SDK2_STATIC_LIB_RELEASE_PATH "${LIVOX_SDK2_STATIC_LIB_RELEASE}")
       set(LIVOX_SDK2_STATIC_LIB_DEBUG_PATH "${LIVOX_SDK2_STATIC_LIB_RELEASE}")
     else()
       message(FATAL_ERROR "Livox SDK2 not found. Install via: git clone + cmake + make install")
     endif()
   else()
     message(FATAL_ERROR "Livox SDK2 not found. Set LIVOX_SDK2_ROOT or install system-wide.")
   endif()
 elseif(EXISTS "${LIVOX_SDK2_ROOT}")
 	# 尝试查找静态 Livox SDK 的 Release 与 Debug 版本（分别为不同文件名）
 	find_library(LIVOX_SDK2_STATIC_LIB_RELEASE NAMES livox_lidar_sdk_static HINTS ${LIVOX_SDK2_ROOT}/lib PATH_SUFFIXES lib)
 	find_library(LIVOX_SDK2_STATIC_LIB_DEBUG   NAMES livox_lidar_sdk_staticd livox_lidar_sdk_static_d HINTS ${LIVOX_SDK2_ROOT}/lib PATH_SUFFIXES lib)
	if(LIVOX_SDK2_STATIC_LIB_RELEASE OR LIVOX_SDK2_STATIC_LIB_DEBUG)
		message(STATUS "Livox SDK2 libs found: Release=${LIVOX_SDK2_STATIC_LIB_RELEASE} Debug=${LIVOX_SDK2_STATIC_LIB_DEBUG}")
		set(LIVOX_SDK2_INCLUDE_DIR "${LIVOX_SDK2_ROOT}/include")
		set(LIVOX_SDK2_STATIC_LIB_RELEASE_PATH "${LIVOX_SDK2_STATIC_LIB_RELEASE}")
		set(LIVOX_SDK2_STATIC_LIB_DEBUG_PATH "${LIVOX_SDK2_STATIC_LIB_DEBUG}")
	else()
		message(FATAL_ERROR "Livox SDK2 library not found in ${LIVOX_SDK2_ROOT}. Please check your installation.")
	endif()
 endif()
endif()

# Eigen3（用于数学运算）
if(NOT DEFINED Eigen3_DIR)
 if(EXISTS "${CMAKE_SOURCE_DIR}/3rdParty/${PCL_VERSION_DIR}/3rdParty/Eigen3/share/eigen3/cmake")
 set(Eigen3_DIR "${CMAKE_SOURCE_DIR}/3rdParty/${PCL_VERSION_DIR}/3rdParty/Eigen3/share/eigen3/cmake")
 endif()
endif()
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
 message(STATUS "Eigen3 found: ${Eigen3_VERSION}")
 list(APPEND EXTRA_LIBS Eigen3::Eigen)
endif()

# 启用 Qt 的自动 moc 与 autouic 功能（仅 GUI 构建需要）
if(USE_QT)
 set(CMAKE_AUTOMOC ON)
 set(CMAKE_AUTOUIC ON)
endif()

if(BUILD_GUI)
 # 收集源码、UI 与头文件列表（递归），并将 UI 文件加入源文件列表
 file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "src/*.cpp" "src/*.cxx" "src/*.cc")
 file(GLOB_RECURSE UI_FILES CONFIGURE_DEPENDS "ui/*.ui" "ui/**/*.ui")
 file(GLOB_RECURSE HDR_FILES CONFIGURE_DEPENDS "include/*.h" "include/**/*.h")
	# 避免将 headless 入口编译进 GUI，可执行里只保留 GUI 的 main
	list(FILTER SRC_FILES EXCLUDE REGEX "main_headless\\.cpp$")
 list(APPEND SRC_FILES ${UI_FILES})

 add_executable(linger_perceiver ${SRC_FILES})

 # 设置 AUTOUIC 的搜索路径为项目的 UI 目录
 set_target_properties(linger_perceiver PROPERTIES AUTOUIC_SEARCH_PATHS "${CMAKE_SOURCE_DIR}/ui")

 # 如果存在头文件列表，则把头文件也作为 target 的源文件（可选）
 if(HDR_FILES)
  target_sources(linger_perceiver PRIVATE ${HDR_FILES})
 endif()

 target_include_directories(linger_perceiver PRIVATE 
     ${CMAKE_CURRENT_SOURCE_DIR}/include 
     ${CMAKE_CURRENT_BINARY_DIR}
     ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/rapidjson
     ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/spdlog
 )

 # 如果存在额外的编译宏定义，则逐个添加到目标
 if(EXTRA_COMPILE_DEFS)
  foreach(def IN LISTS EXTRA_COMPILE_DEFS)
  target_compile_definitions(linger_perceiver PRIVATE ${def})
  endforeach()
 endif()

 # 支持同时生成 Debug 与 Release：为每个配置设置单独的输出目录
 # 并为不同配置添加相应的编译宏与编译选项。
 # 对于单配置的生成器（如 Makefile / Ninja），允许通过 `CMAKE_BUILD_TYPE` 选择构建类型。
 if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
 endif()

 set_target_properties(linger_perceiver PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
 )

 # 配置特定的编译宏定义（通过宏暴露给源代码）
 target_compile_definitions(linger_perceiver PRIVATE
	$<$<CONFIG:Debug>:DEBUG_BUILD>
	$<$<CONFIG:RelWithDebInfo>:DEBUG_BUILD>
	$<$<CONFIG:Release>:NDEBUG>
 )

 # 针对不同编译器的配置特定编译选项（MSVC / gcc / clang）
 if(MSVC)
	target_compile_options(linger_perceiver PRIVATE
		$<$<CONFIG:Debug>:/Zi;/Od;/RTC1;/FS>
		$<$<CONFIG:Release>:/O2;/DNDEBUG;/FS>
	)
 else()
	target_compile_options(linger_perceiver PRIVATE
		$<$<CONFIG:Debug>:-g;-O0>
		$<$<CONFIG:Release>:-O3;-DNDEBUG>
	)
 endif()
	install(TARGETS linger_perceiver RUNTIME DESTINATION bin)

	# 复制配置文件到输出目录的 config 子目录
	add_custom_command(TARGET linger_perceiver POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:linger_perceiver>/config"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/config/mid360_config.json"
		"$<TARGET_FILE_DIR:linger_perceiver>/config/mid360_config.json"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/config/app_config.json"
		"$<TARGET_FILE_DIR:linger_perceiver>/config/app_config.json"
		COMMENT "Copying config files to output directory"
	)

	# Qt 翻译文件处理
	if(ENABLE_TRANSLATIONS AND Qt5LinguistTools_FOUND)
		# 定义翻译文件列表
		set(TS_FILES
			"${CMAKE_SOURCE_DIR}/translations/linger_zh_CN.ts"
			"${CMAKE_SOURCE_DIR}/translations/linger_en.ts"
		)
		
		# 定义需要扫描的源文件（用于 lupdate）
		# 使用目录而非文件列表，以便递归扫描所有源文件
		set(TRANSLATION_SOURCE_DIRS
			${CMAKE_SOURCE_DIR}/src
			${CMAKE_SOURCE_DIR}/include
			${CMAKE_SOURCE_DIR}/ui
		)
		
		# 添加 update_translations 目标（跨平台的 lupdate 调用）
		# 使用方法: cmake --build . --target update_translations
		get_target_property(LUPDATE_EXECUTABLE Qt5::lupdate IMPORTED_LOCATION)
		if(LUPDATE_EXECUTABLE)
			add_custom_target(update_translations
				COMMAND ${LUPDATE_EXECUTABLE}
					-locations relative
					-no-obsolete
					${TRANSLATION_SOURCE_DIRS}
					-ts ${TS_FILES}
				WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
				COMMENT "Updating translation files (.ts) with lupdate..."
				VERBATIM
			)
			message(STATUS "Target 'update_translations' available - run: cmake --build . --target update_translations")
		else()
			message(WARNING "lupdate not found - update_translations target not available")
		endif()
		
		# 设置 .qm 文件输出目录
		set(QM_OUTPUT_DIR "${CMAKE_BINARY_DIR}/translations")
		file(MAKE_DIRECTORY ${QM_OUTPUT_DIR})
		
		# 使用 qt5_add_translation 生成 .qm 文件
		qt5_add_translation(QM_FILES ${TS_FILES})
		
		# 将 .qm 文件作为目标源文件，确保被构建
		target_sources(linger_perceiver PRIVATE ${QM_FILES})
		
		# 添加翻译相关的编译宏
		target_compile_definitions(linger_perceiver PRIVATE ENABLE_TRANSLATIONS)
		
		# 复制 .qm 文件到输出目录的 translations 子目录
		add_custom_command(TARGET linger_perceiver POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:linger_perceiver>/translations"
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${QM_FILES} "$<TARGET_FILE_DIR:linger_perceiver>/translations/"
			COMMENT "Copying translation files (.qm) to output directory"
		)
		
		message(STATUS "Translation files: ${TS_FILES}")
	endif()
endif() # BUILD_GUI

# GUI 目标后处理与链接
if(BUILD_GUI)
	# 如果有额外的库列表（VTK/PCL 等），将其链接到可执行目标
	if(EXTRA_LIBS)
	 target_link_libraries(linger_perceiver PRIVATE ${EXTRA_LIBS})
	endif()

	# 根据 VTK_DIR 推断 VTK 安装的 bin 目录（用于部署运行时 DLL）
	 if(DEFINED VTK_DIR)
	 get_filename_component(_vtk_lib_dir "${VTK_DIR}" DIRECTORY) # VTK 的 lib/cmake 目录
	 get_filename_component(_vtk_lib_dir "${_vtk_lib_dir}" DIRECTORY) # VTK 的 lib 目录
	 get_filename_component(_vtk_prefix "${_vtk_lib_dir}" DIRECTORY) # VTK 安装前缀（install prefix）
	 set(_vtk_bin_dir "${_vtk_prefix}/bin")
	 if(NOT EXISTS "${_vtk_bin_dir}")
	 set(_vtk_bin_dir "${VTK_DIR}/../bin")
	 endif()
	else()
	 set(_vtk_bin_dir "")
	endif()

	message(STATUS "Guessed VTK bin dir: ${_vtk_bin_dir}")

	# 在 Windows 平台：若已找到 VTK 且 bin 目录存在，添加 post-build 命令将 VTK 运行时 DLL 部署到可执行文件目录
	if(WIN32 AND VTK_FOUND AND EXISTS "${_vtk_bin_dir}")
	 add_custom_command(TARGET linger_perceiver POST_BUILD
	  COMMAND ${CMAKE_COMMAND} -E echo "Copying VTK runtime from ${_vtk_bin_dir} to $<TARGET_FILE_DIR:linger_perceiver> (config: $<CONFIG>)"
	  COMMAND ${CMAKE_COMMAND} -Dsrc_dir=${_vtk_bin_dir} -Ddst_dir=$<TARGET_FILE_DIR:linger_perceiver> -Dcfg=$<CONFIG> -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CopyVtkDlls.cmake
	  COMMENT "Deploy VTK runtime DLLs for current configuration"
	  VERBATIM)
	elseif(WIN32)
	 message(WARNING "VTK bin dir not found; cannot deploy VTK runtime DLLs automatically.")
	endif()

	# 自动复制 PCL DLLs (Windows)
	if(WIN32 AND USE_PCL)
		# 使用明确定义的路径，避免 PCL_DIR 被 find_package 修改导致路径错误
		set(_pcl_bin_dir "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/bin")
        
		if(EXISTS "${_pcl_bin_dir}")
			# 使用 cmake 脚本根据配置(Debug/Release) 智能拷贝 PCL DLL
			add_custom_command(TARGET linger_perceiver POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E echo "Copying PCL runtime from ${_pcl_bin_dir} to $<TARGET_FILE_DIR:linger_perceiver> (config: $<CONFIG>)"
				COMMAND ${CMAKE_COMMAND} -Dsrc_dir=${_pcl_bin_dir} -Ddst_dir=$<TARGET_FILE_DIR:linger_perceiver> -Dcfg=$<CONFIG> -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CopyPclDlls.cmake
				COMMENT "Deploy PCL runtime DLLs for current configuration"
				VERBATIM
			)
		else()
			message(WARNING "PCL bin directory not found at: ${_pcl_bin_dir}. You may need to copy PCL DLLs manually.")
		endif()

		# 检查 OpenNI2 (PCL 常见依赖)
		set(_openni2_bin_dir "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/3rdParty/OpenNI2/Tools")
		if(EXISTS "${_openni2_bin_dir}")
			file(GLOB _openni2_dlls "${_openni2_bin_dir}/*.dll")
			foreach(_dll ${_openni2_dlls})
				add_custom_command(TARGET linger_perceiver POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${_dll}"
					"$<TARGET_FILE_DIR:linger_perceiver>"
					VERBATIM
				)
			endforeach()
		endif()
	endif()

	# 如果启用了 Qt 支持且找到了 Qt Widgets，链接 Qt 库并包含生成目录（供 ui/moc 产物使用）
	if(USE_QT AND Qt5Widgets_FOUND)
	 target_link_libraries(linger_perceiver PRIVATE Qt5::Widgets)
	 target_include_directories(linger_perceiver PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
	endif()

	# Windows 平台相关的运行时部署（windeployqt、插件复制等）
	if(WIN32)
	# 以下为 Windows 平台下的 Qt 部署与插件处理逻辑
	set(_first_qt_prefix "")
	if(CMAKE_PREFIX_PATH)
	 list(GET CMAKE_PREFIX_PATH 0 _first_qt_prefix)
	endif()
	set(_qt_bin_dir "${_first_qt_prefix}/bin")
	set(_windeployqt_args "")

	find_program(WINDEPLOYQT_EXE NAMES windeployqt HINTS "${_qt_bin_dir}" PATHS ENV PATH)

	if(WINDEPLOYQT_EXE)
	 add_custom_command(TARGET linger_perceiver POST_BUILD
	 COMMAND "${WINDEPLOYQT_EXE}" ${_windeployqt_args} "$<TARGET_FILE:linger_perceiver>"
	 COMMENT "Running windeployqt to deploy Qt runtime"
	 VERBATIM)
	else()
	 message(WARNING "windeployqt not found; Qt DLLs will not be deployed automatically.")
	endif()

	# 设置 Qt 插件目录以便复制 platform 插件等运行时资源
	set(_qt_plugins_dir "${_first_qt_prefix}/plugins")

	if(EXISTS "${_qt_plugins_dir}/platforms")
	 add_custom_command(TARGET linger_perceiver POST_BUILD
	 COMMAND ${CMAKE_COMMAND} -E copy_directory
	 "${_qt_plugins_dir}/platforms"
	 "$<TARGET_FILE_DIR:linger_perceiver>/platforms"
	 COMMENT "Copy Qt platform plugins (qwindows) to output")
	else()
	 message(WARNING "Qt plugins/platforms not found at ${_qt_plugins_dir}/platforms")
	endif()

	# 在 Windows 上，如果启用了 Qt 并找到 Qt5::Core，可尝试将 Qt 插件复制到可执行目录
	if(USE_QT AND Qt5Widgets_FOUND)
	 add_custom_command(TARGET linger_perceiver POST_BUILD
	 COMMAND ${CMAKE_COMMAND} -E echo "Deploying Qt plugins from $<TARGET_FILE_DIR:Qt5::Core>/../plugins to $<TARGET_FILE_DIR:linger_perceiver>"
	 COMMAND ${CMAKE_COMMAND} -E copy_directory "$<TARGET_FILE_DIR:Qt5::Core>/../plugins" "$<TARGET_FILE_DIR:linger_perceiver>/plugins"
	 COMMAND ${CMAKE_COMMAND} -E copy_directory "$<TARGET_FILE_DIR:Qt5::Core>/../plugins/platforms" "$<TARGET_FILE_DIR:linger_perceiver>/platforms"
	 COMMENT "Copy Qt plugins (fallback via Qt target path)"
	 VERBATIM)
	endif()
	endif() # WIN32

endif()

# Headless 模式可选构建：不依赖 Qt/VTK，仅保留设备/处理/通信管线
if(BUILD_HEADLESS)
	set(HEADLESS_SOURCES
		src/main_headless.cpp
		src/acquisition/device_manager.cpp
		src/acquisition/data_processor.cpp
		src/core/processor.cpp
		src/core/app_config.cpp
		src/core/app_init.cpp
		src/acquisition/replayer.cpp
		src/acquisition/streamer.cpp
		src/preprocessing/distance_filter.cpp
		src/preprocessing/voxel_filter.cpp
		src/preprocessing/crop_box_filter.cpp
		src/preprocessing/statistical_outlier_filter.cpp
		src/preprocessing/density_filter.cpp
		src/preprocessing/motion_filter.cpp
		src/segmentation/sea_surface_filter.cpp
		src/algorithms/docking_algorithm.cpp
		src/communication/docking_publisher.cpp
	)

	add_executable(linger_perceiver_service ${HEADLESS_SOURCES})

	target_include_directories(linger_perceiver_service PRIVATE 
		${CMAKE_CURRENT_SOURCE_DIR}/include 
		${CMAKE_CURRENT_BINARY_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/rapidjson
		${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/spdlog
	)

	# 编译宏：与 GUI 目标保持一致，并标记 HEADLESS_MODE
	target_compile_definitions(linger_perceiver_service PRIVATE
		$<$<CONFIG:Debug>:DEBUG_BUILD>
		$<$<CONFIG:RelWithDebInfo>:DEBUG_BUILD>
		$<$<CONFIG:Release>:NDEBUG>
		HEADLESS_MODE
	)

	# 编译选项：与 GUI 目标保持一致
	if(MSVC)
		target_compile_options(linger_perceiver_service PRIVATE
			$<$<CONFIG:Debug>:/Zi;/Od;/RTC1;/FS>
			$<$<CONFIG:Release>:/O2;/DNDEBUG;/FS>
		)
	else()
		target_compile_options(linger_perceiver_service PRIVATE
			$<$<CONFIG:Debug>:-g;-O0>
			$<$<CONFIG:Release>:-O3;-DNDEBUG>
		)
	endif()

	set_target_properties(linger_perceiver_service PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
		ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
		LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
	)

	# 链接仅需 PCL/Eigen/Livox（无 Qt/VTK）+ WinSock（UDP 通信）
	set(HEADLESS_LIBS "")
	if(USE_PCL)
		list(APPEND HEADLESS_LIBS ${PCL_LIBRARIES})
	endif()
	if(Eigen3_FOUND)
		list(APPEND HEADLESS_LIBS Eigen3::Eigen)
	endif()
	if(DEFINED LIVOX_SDK2_INCLUDE_DIR)
		target_include_directories(linger_perceiver_service PRIVATE ${LIVOX_SDK2_INCLUDE_DIR})
	endif()
	# Livox SDK2 链接：Linux 使用统一库，Windows 区分 Debug/Release
	if(UNIX AND NOT APPLE)
		if(DEFINED LIVOX_SDK2_STATIC_LIB_RELEASE_PATH)
			list(APPEND HEADLESS_LIBS ${LIVOX_SDK2_STATIC_LIB_RELEASE_PATH})
		endif()
	else()
		if(DEFINED LIVOX_SDK2_STATIC_LIB_DEBUG_PATH)
			list(APPEND HEADLESS_LIBS $<$<CONFIG:Debug>:${LIVOX_SDK2_STATIC_LIB_DEBUG_PATH}>)
		endif()
		if(DEFINED LIVOX_SDK2_STATIC_LIB_RELEASE_PATH)
			list(APPEND HEADLESS_LIBS $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${LIVOX_SDK2_STATIC_LIB_RELEASE_PATH}>)
		endif()
	endif()
	# Windows 平台需要链接 WinSock2（用于 UDP 通信）
	if(WIN32)
		list(APPEND HEADLESS_LIBS ws2_32)
	endif()
	
	# Linux 平台需要链接 pthread
	if(UNIX AND NOT APPLE)
		list(APPEND HEADLESS_LIBS pthread)
	endif()

	if(HEADLESS_LIBS)
		target_link_libraries(linger_perceiver_service PRIVATE ${HEADLESS_LIBS})
	endif()

	# 复制配置文件到输出目录的 config 子目录，便于直接运行
	add_custom_command(TARGET linger_perceiver_service POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:linger_perceiver_service>/config"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/config/mid360_config.json"
		"$<TARGET_FILE_DIR:linger_perceiver_service>/config/mid360_config.json"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${CMAKE_SOURCE_DIR}/config/app_config.json"
		"$<TARGET_FILE_DIR:linger_perceiver_service>/config/app_config.json"
		COMMENT "Copying config files to headless output directory"
	)

	# 自动复制 PCL DLLs (Windows) - Headless 模式也需要 PCL 运行时
	if(WIN32 AND USE_PCL)
		set(_pcl_bin_dir "${LOCAL_3RDPARTY_DIR}/${PCL_VERSION_DIR}/bin")
		if(EXISTS "${_pcl_bin_dir}")
			add_custom_command(TARGET linger_perceiver_service POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E echo "Copying PCL runtime from ${_pcl_bin_dir} to $<TARGET_FILE_DIR:linger_perceiver_service> (config: $<CONFIG>)"
				COMMAND ${CMAKE_COMMAND} -Dsrc_dir=${_pcl_bin_dir} -Ddst_dir=$<TARGET_FILE_DIR:linger_perceiver_service> -Dcfg=$<CONFIG> -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CopyPclDlls.cmake
				COMMENT "Deploy PCL runtime DLLs for headless configuration"
				VERBATIM
			)
		else()
			message(WARNING "PCL bin directory not found at: ${_pcl_bin_dir}. Headless service may need PCL DLLs copied manually.")
		endif()
	endif()

	install(TARGETS linger_perceiver_service RUNTIME DESTINATION bin)
endif()

# 对 Livox SDK 进行按配置链接，避免不同配置间的 CRT 不匹配问题（仅 GUI 目标存在时）
if(BUILD_GUI AND DEFINED LIVOX_SDK2_INCLUDE_DIR)
	target_include_directories(linger_perceiver PRIVATE ${LIVOX_SDK2_INCLUDE_DIR})
	# Debug library
	if(DEFINED LIVOX_SDK2_STATIC_LIB_DEBUG_PATH)
		target_link_libraries(linger_perceiver PRIVATE
			$<$<CONFIG:Debug>:${LIVOX_SDK2_STATIC_LIB_DEBUG_PATH}>
		)
	endif()
	# Release / RelWithDebInfo library
	if(DEFINED LIVOX_SDK2_STATIC_LIB_RELEASE_PATH)
		target_link_libraries(linger_perceiver PRIVATE
			$<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${LIVOX_SDK2_STATIC_LIB_RELEASE_PATH}>
		)
	endif()
	message(STATUS "Configured Livox SDK linking: Debug=${LIVOX_SDK2_STATIC_LIB_DEBUG_PATH} Release=${LIVOX_SDK2_STATIC_LIB_RELEASE_PATH}")
endif()

# 输出当前的配置摘要，便于构建日志查看
message(STATUS "Configuration summary:")
message(STATUS " USE_PCL = ${USE_PCL}")
message(STATUS " USE_QT = ${USE_QT}")
message(STATUS " USE_LIVOX = ${USE_LIVOX}")
message(STATUS " BUILD_GUI = ${BUILD_GUI}")
message(STATUS " BUILD_HEADLESS = ${BUILD_HEADLESS}")
message(STATUS " ENABLE_TRANSLATIONS = ${ENABLE_TRANSLATIONS}")
message(STATUS " PCL_DIR = ${PCL_DIR}")
message(STATUS " QT_DIR = ${QT_DIR}")
message(STATUS " LIVOX_SDK2_ROOT = ${LIVOX_SDK2_ROOT}")
