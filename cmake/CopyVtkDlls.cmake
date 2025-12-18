## CopyVtkDlls.cmake
#
# 用法: 通过 `cmake -Dsrc_dir=... -Ddst_dir=... -Dcfg=<Config> -P CopyVtkDlls.cmake` 调用
# 本脚本会根据传入的 `cfg` 配置字符串，从提供的 `src_dir` 中
# 复制 VTK 运行时的 DLL 到 `dst_dir`。脚本会区分 Debug（文件名以 'd.dll' 结尾）
# 与 Release（其它 .dll）版本，只复制对应配置的 DLL 文件。

if(NOT DEFINED src_dir)
  message(FATAL_ERROR "CopyVtkDlls.cmake: src_dir not specified")
endif()
if(NOT DEFINED dst_dir)
  message(FATAL_ERROR "CopyVtkDlls.cmake: dst_dir not specified")
endif()
if(NOT DEFINED cfg)
  set(cfg "")
endif()

# 去除可能在自定义 COMMAND 调用中注入的外层引号（某些生成器会传入带引号的参数）。
# 这样可以提高脚本在通过 CMake 自定义命令调用时的鲁棒性。
string(REPLACE '"' "" src_dir "${src_dir}")
string(REPLACE '"' "" dst_dir "${dst_dir}")

# 如果 src/dst 仍被字面上的首尾引号包裹（例如 '"C:/path"'），
# 则通过子串安全地去除这些引号。
string(LENGTH src_dir _src_len)
if(_src_len GREATER 1)
  string(SUBSTRING src_dir 0 1 _src_first)
  math(EXPR _src_last_idx "${_src_len} - 1")
  string(SUBSTRING src_dir ${_src_last_idx} 1 _src_last)
  if(_src_first STREQUAL '"' AND _src_last STREQUAL '"')
    string(SUBSTRING src_dir 1 ${_src_last_idx} src_dir)
  endif()
endif()

string(LENGTH dst_dir _dst_len)
if(_dst_len GREATER 1)
  string(SUBSTRING dst_dir 0 1 _dst_first)
  math(EXPR _dst_last_idx "${_dst_len} - 1")
  string(SUBSTRING dst_dir ${_dst_last_idx} 1 _dst_last)
  if(_dst_first STREQUAL '"' AND _dst_last STREQUAL '"')
    string(SUBSTRING dst_dir 1 ${_dst_last_idx} dst_dir)
  endif()
endif()

file(GLOB VTK_DLLS "${src_dir}/*.dll")
message(STATUS "CopyVtkDlls: src_dir='${src_dir}'")
if(NOT EXISTS "${src_dir}")
  message(STATUS "CopyVtkDlls: src_dir does not exist: ${src_dir}")
endif()
list(LENGTH VTK_DLLS _vcount)
message(STATUS "CopyVtkDlls: glob found ${_vcount} entries in ${src_dir}")
if(_vcount EQUAL 0)
  message(STATUS "CopyVtkDlls: no .dll files found in ${src_dir}")
  return()
endif()

set(DEBUG_LIST "")
set(RELEASE_LIST "")
foreach(f ${VTK_DLLS})
  get_filename_component(fname "${f}" NAME)
  # 将以 'd.dll' 结尾的文件名视为 Debug 版本
  string(REGEX MATCH "d\\.dll$" is_debug "${fname}")
  if(is_debug)
    list(APPEND DEBUG_LIST "${f}")
  else()
    list(APPEND RELEASE_LIST "${f}")
  endif()
endforeach()

if("${cfg}" STREQUAL "Debug" OR "${cfg}" STREQUAL "RelWithDebInfo")
  set(TO_COPY ${DEBUG_LIST})
else()
  set(TO_COPY ${RELEASE_LIST})
endif()

if(NOT TO_COPY)
  message(STATUS "CopyVtkDlls: nothing to copy for configuration '${cfg}'")
  return()
endif()

file(MAKE_DIRECTORY "${dst_dir}")
foreach(f ${TO_COPY})
  message(STATUS "CopyVtkDlls: copying ${f} -> ${dst_dir}")
  file(COPY "${f}" DESTINATION "${dst_dir}")
endforeach()
