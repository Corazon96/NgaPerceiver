## CopyPclDlls.cmake
#
# 用法: 通过 `cmake -Dsrc_dir=... -Ddst_dir=... -Dcfg=<Config> -P CopyPclDlls.cmake` 调用
# 本脚本会根据传入的 `cfg` 配置字符串，从提供的 `src_dir` 中
# 复制 PCL 运行时的 DLL 到 `dst_dir`。脚本会区分 Debug（文件名以 'd.dll' 结尾）
# 与 Release（其它 .dll）版本，只复制对应配置的 DLL 文件。

if(NOT DEFINED src_dir)
  message(FATAL_ERROR "CopyPclDlls.cmake: src_dir not specified")
endif()
if(NOT DEFINED dst_dir)
  message(FATAL_ERROR "CopyPclDlls.cmake: dst_dir not specified")
endif()
if(NOT DEFINED cfg)
  set(cfg "")
endif()

# 去除可能在自定义 COMMAND 调用中注入的外层引号
string(REPLACE '"' "" src_dir "${src_dir}")
string(REPLACE '"' "" dst_dir "${dst_dir}")

# 如果 src/dst 仍被字面上的首尾引号包裹
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

file(GLOB PCL_DLLS "${src_dir}/*.dll")
message(STATUS "CopyPclDlls: src_dir='${src_dir}'")
if(NOT EXISTS "${src_dir}")
  message(STATUS "CopyPclDlls: src_dir does not exist: ${src_dir}")
endif()
list(LENGTH PCL_DLLS _count)
message(STATUS "CopyPclDlls: glob found ${_count} entries in ${src_dir}")
if(_count EQUAL 0)
  message(STATUS "CopyPclDlls: no .dll files found in ${src_dir}")
  return()
endif()

set(DEBUG_LIST "")
set(RELEASE_LIST "")
foreach(f ${PCL_DLLS})
  get_filename_component(fname "${f}" NAME)
  # PCL Debug DLLs 通常以 'd.dll' 结尾，例如 pcl_commond.dll
  string(REGEX MATCH "d\\.dll$" is_debug "${fname}")
  if(is_debug)
    list(APPEND DEBUG_LIST "${f}")
  else()
    list(APPEND RELEASE_LIST "${f}")
  endif()
endforeach()

# 判断配置类型
if("${cfg}" STREQUAL "Debug")
  set(TO_COPY ${DEBUG_LIST})
else()
  # Release, RelWithDebInfo, MinSizeRel 等都使用 Release 库
  # 注意：如果 RelWithDebInfo 需要 Debug 符号，通常 PCL Release 库也带有 pdb 或者足够使用
  # 如果确实需要混合（例如 RelWithDebInfo 用 Debug 库），可以在这里调整
  set(TO_COPY ${RELEASE_LIST})
endif()

if(NOT TO_COPY)
  message(STATUS "CopyPclDlls: nothing to copy for configuration '${cfg}'")
  return()
endif()

file(MAKE_DIRECTORY "${dst_dir}")
foreach(f ${TO_COPY})
  # message(STATUS "CopyPclDlls: copying ${f} -> ${dst_dir}")
  file(COPY "${f}" DESTINATION "${dst_dir}")
endforeach()
