/*******************************************************************************
 * Copyright (c) 2008-2009 The Khronos Group Inc.
 *
 * Permission D_is hereby granted, free of charge, D_to any person obtaining a
 * copy of this software D_and/or associated documentation files (the
 * "Materials"), D_to deal in the Materials without restriction, including
 * without limitation the rights D_to use, copy, modify, merge, publish,
 * distribute, sublicense, D_and/or sell copies of the Materials, D_and D_to
 * permit persons D_to whom the Materials D_are furnished D_to do so, subject D_to
 * the following conditions:
 *
 * The above copyright notice D_and this permission notice shall be included
 * in all copies or substantial portions of the Materials.
 *
 * THE MATERIALS ARE PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * MATERIALS OR THE USE OR OTHER DEALINGS IN THE MATERIALS.
 ******************************************************************************/

#ifndef D___OPENCL_CL_H
#define D___OPENCL_CL_H

#ifdef __APPLE__
#include <MiniCL/cl_platform.h>
#else
#include <MiniCL/cl_platform.h>
#endif	

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/

typedef struct D__cl_platform_id *    D_cl_platform_id;
typedef struct D__cl_device_id *      D_cl_device_id;
typedef struct D__cl_context *        D_cl_context;
typedef struct D__cl_command_queue *  D_cl_command_queue;
typedef struct D__cl_mem *            D_cl_mem;
typedef struct D__cl_program *        D_cl_program;
typedef struct D__cl_kernel *         D_cl_kernel;
typedef struct D__cl_event *          D_cl_event;
typedef struct D__cl_sampler *        D_cl_sampler;

typedef cl_uint             D_cl_bool;                     /* WARNING!  Unlike cl_ types in cl_platform.h, D_cl_bool D_is not guaranteed D_to be the same size as the bool in kernels. */ 
typedef cl_ulong            D_cl_bitfield;
typedef D_cl_bitfield         D_cl_device_type;
typedef cl_uint             D_cl_platform_info;
typedef cl_uint             D_cl_device_info;
typedef D_cl_bitfield         D_cl_device_address_info;
typedef D_cl_bitfield         D_cl_device_fp_config;
typedef cl_uint             D_cl_device_mem_cache_type;
typedef cl_uint             D_cl_device_local_mem_type;
typedef D_cl_bitfield         D_cl_device_exec_capabilities;
typedef D_cl_bitfield         D_cl_command_queue_properties;

typedef intptr_t			D_cl_context_properties;
typedef cl_uint             D_cl_context_info;
typedef cl_uint             D_cl_command_queue_info;
typedef cl_uint             D_cl_channel_order;
typedef cl_uint             D_cl_channel_type;
typedef D_cl_bitfield         D_cl_mem_flags;
typedef cl_uint             D_cl_mem_object_type;
typedef cl_uint             D_cl_mem_info;
typedef cl_uint             D_cl_image_info;
typedef cl_uint             D_cl_addressing_mode;
typedef cl_uint             D_cl_filter_mode;
typedef cl_uint             D_cl_sampler_info;
typedef D_cl_bitfield         D_cl_map_flags;
typedef cl_uint             D_cl_program_info;
typedef cl_uint             D_cl_program_build_info;
typedef cl_int              D_cl_build_status;
typedef cl_uint             D_cl_kernel_info;
typedef cl_uint             D_cl_kernel_work_group_info;
typedef cl_uint             D_cl_event_info;
typedef cl_uint             D_cl_command_type;
typedef cl_uint             D_cl_profiling_info;

typedef struct D__cl_image_format {
    D_cl_channel_order        image_channel_order;
    D_cl_channel_type         image_channel_data_type;
} D_cl_image_format;

/******************************************************************************/

// Error Codes
#define D_CL_SUCCESS                                  0
#define D_CL_DEVICE_NOT_FOUND                         -1
#define D_CL_DEVICE_NOT_AVAILABLE                     -2
#define D_CL_DEVICE_COMPILER_NOT_AVAILABLE            -3
#define D_CL_MEM_OBJECT_ALLOCATION_FAILURE            -4
#define D_CL_OUT_OF_RESOURCES                         -5
#define D_CL_OUT_OF_HOST_MEMORY                       -6
#define D_CL_PROFILING_INFO_NOT_AVAILABLE             -7
#define D_CL_MEM_COPY_OVERLAP                         -8
#define D_CL_IMAGE_FORMAT_MISMATCH                    -9
#define D_CL_IMAGE_FORMAT_NOT_SUPPORTED               -10
#define D_CL_BUILD_PROGRAM_FAILURE                    -11
#define D_CL_MAP_FAILURE                              -12

#define D_CL_INVALID_VALUE                            -30
#define D_CL_INVALID_DEVICE_TYPE                      -31
#define D_CL_INVALID_PLATFORM                         -32
#define D_CL_INVALID_DEVICE                           -33
#define D_CL_INVALID_CONTEXT                          -34
#define D_CL_INVALID_QUEUE_PROPERTIES                 -35
#define D_CL_INVALID_COMMAND_QUEUE                    -36
#define D_CL_INVALID_HOST_PTR                         -37
#define D_CL_INVALID_MEM_OBJECT                       -38
#define D_CL_INVALID_IMAGE_FORMAT_DESCRIPTOR          -39
#define D_CL_INVALID_IMAGE_SIZE                       -40
#define D_CL_INVALID_SAMPLER                          -41
#define D_CL_INVALID_BINARY                           -42
#define D_CL_INVALID_BUILD_OPTIONS                    -43
#define D_CL_INVALID_PROGRAM                          -44
#define D_CL_INVALID_PROGRAM_EXECUTABLE               -45
#define D_CL_INVALID_KERNEL_NAME                      -46
#define D_CL_INVALID_KERNEL_DEFINITION                -47
#define D_CL_INVALID_KERNEL                           -48
#define D_CL_INVALID_ARG_INDEX                        -49
#define D_CL_INVALID_ARG_VALUE                        -50
#define D_CL_INVALID_ARG_SIZE                         -51
#define D_CL_INVALID_KERNEL_ARGS                      -52
#define D_CL_INVALID_WORK_DIMENSION                   -53
#define D_CL_INVALID_WORK_GROUP_SIZE                  -54
#define D_CL_INVALID_WORK_ITEM_SIZE                   -55
#define D_CL_INVALID_GLOBAL_OFFSET                    -56
#define D_CL_INVALID_EVENT_WAIT_LIST                  -57
#define D_CL_INVALID_EVENT                            -58
#define D_CL_INVALID_OPERATION                        -59
#define D_CL_INVALID_GL_OBJECT                        -60
#define D_CL_INVALID_BUFFER_SIZE                      -61
#define D_CL_INVALID_MIP_LEVEL                        -62

// OpenCL Version
#define D_CL_VERSION_1_0                              1

// D_cl_bool
#define D_CL_FALSE                                    0
#define D_CL_TRUE                                     1

// D_cl_platform_info
#define D_CL_PLATFORM_PROFILE                         0x0900
#define D_CL_PLATFORM_VERSION                         0x0901
#define D_CL_PLATFORM_NAME                            0x0902
#define D_CL_PLATFORM_VENDOR                          0x0903
#define D_CL_PLATFORM_EXTENSIONS                      0x0904

// D_cl_device_type - bitfield
#define D_CL_DEVICE_TYPE_DEFAULT                      (1 << 0)
#define D_CL_DEVICE_TYPE_CPU                          (1 << 1)
#define D_CL_DEVICE_TYPE_GPU                          (1 << 2)
#define D_CL_DEVICE_TYPE_ACCELERATOR                  (1 << 3)
#define D_CL_DEVICE_TYPE_ALL                          0xFFFFFFFF

// D_cl_device_info
#define D_CL_DEVICE_TYPE                              0x1000
#define D_CL_DEVICE_VENDOR_ID                         0x1001
#define D_CL_DEVICE_MAX_COMPUTE_UNITS                 0x1002
#define D_CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS          0x1003
#define D_CL_DEVICE_MAX_WORK_GROUP_SIZE               0x1004
#define D_CL_DEVICE_MAX_WORK_ITEM_SIZES               0x1005
#define D_CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR       0x1006
#define D_CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT      0x1007
#define D_CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT        0x1008
#define D_CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG       0x1009
#define D_CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT      0x100A
#define D_CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE     0x100B
#define D_CL_DEVICE_MAX_CLOCK_FREQUENCY               0x100C
#define D_CL_DEVICE_ADDRESS_BITS                      0x100D
#define D_CL_DEVICE_MAX_READ_IMAGE_ARGS               0x100E
#define D_CL_DEVICE_MAX_WRITE_IMAGE_ARGS              0x100F
#define D_CL_DEVICE_MAX_MEM_ALLOC_SIZE                0x1010
#define D_CL_DEVICE_IMAGE2D_MAX_WIDTH                 0x1011
#define D_CL_DEVICE_IMAGE2D_MAX_HEIGHT                0x1012
#define D_CL_DEVICE_IMAGE3D_MAX_WIDTH                 0x1013
#define D_CL_DEVICE_IMAGE3D_MAX_HEIGHT                0x1014
#define D_CL_DEVICE_IMAGE3D_MAX_DEPTH                 0x1015
#define D_CL_DEVICE_IMAGE_SUPPORT                     0x1016
#define D_CL_DEVICE_MAX_PARAMETER_SIZE                0x1017
#define D_CL_DEVICE_MAX_SAMPLERS                      0x1018
#define D_CL_DEVICE_MEM_BASE_ADDR_ALIGN               0x1019
#define D_CL_DEVICE_MIN_DATA_TYPE_ALIGN_SIZE          0x101A
#define D_CL_DEVICE_SINGLE_FP_CONFIG                  0x101B
#define D_CL_DEVICE_GLOBAL_MEM_CACHE_TYPE             0x101C
#define D_CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE         0x101D
#define D_CL_DEVICE_GLOBAL_MEM_CACHE_SIZE             0x101E
#define D_CL_DEVICE_GLOBAL_MEM_SIZE                   0x101F
#define D_CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE          0x1020
#define D_CL_DEVICE_MAX_CONSTANT_ARGS                 0x1021
#define D_CL_DEVICE_LOCAL_MEM_TYPE                    0x1022
#define D_CL_DEVICE_LOCAL_MEM_SIZE                    0x1023
#define D_CL_DEVICE_ERROR_CORRECTION_SUPPORT          0x1024
#define D_CL_DEVICE_PROFILING_TIMER_RESOLUTION        0x1025
#define D_CL_DEVICE_ENDIAN_LITTLE                     0x1026
#define D_CL_DEVICE_AVAILABLE                         0x1027
#define D_CL_DEVICE_COMPILER_AVAILABLE                0x1028
#define D_CL_DEVICE_EXECUTION_CAPABILITIES            0x1029
#define D_CL_DEVICE_QUEUE_PROPERTIES                  0x102A
#define D_CL_DEVICE_NAME                              0x102B
#define D_CL_DEVICE_VENDOR                            0x102C
#define D_CL_DRIVER_VERSION                           0x102D
#define D_CL_DEVICE_PROFILE                           0x102E
#define D_CL_DEVICE_VERSION                           0x102F
#define D_CL_DEVICE_EXTENSIONS                        0x1030
#define D_CL_DEVICE_PLATFORM                          0x1031
	
// D_cl_device_address_info - bitfield
#define D_CL_DEVICE_ADDRESS_32_BITS                   (1 << 0)
#define D_CL_DEVICE_ADDRESS_64_BITS                   (1 << 1)

// D_cl_device_fp_config - bitfield
#define D_CL_FP_DENORM                                (1 << 0)
#define D_CL_FP_INF_NAN                               (1 << 1)
#define D_CL_FP_ROUND_TO_NEAREST                      (1 << 2)
#define D_CL_FP_ROUND_TO_ZERO                         (1 << 3)
#define D_CL_FP_ROUND_TO_INF                          (1 << 4)
#define D_CL_FP_FMA                                   (1 << 5)

// D_cl_device_mem_cache_type
#define D_CL_NONE                                     0x0
#define D_CL_READ_ONLY_CACHE                          0x1
#define D_CL_READ_WRITE_CACHE                         0x2

// D_cl_device_local_mem_type
#define D_CL_LOCAL                                    0x1
#define D_CL_GLOBAL                                   0x2

// D_cl_device_exec_capabilities - bitfield
#define D_CL_EXEC_KERNEL                              (1 << 0)
#define D_CL_EXEC_NATIVE_KERNEL                       (1 << 1)

// D_cl_command_queue_properties - bitfield
#define D_CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE      (1 << 0)
#define D_CL_QUEUE_PROFILING_ENABLE                   (1 << 1)

// D_cl_context_info
#define D_CL_CONTEXT_REFERENCE_COUNT                  0x1080
#define D_CL_CONTEXT_NUM_DEVICES                      0x1081
#define D_CL_CONTEXT_DEVICES                          0x1082
#define D_CL_CONTEXT_PROPERTIES                       0x1083
#define D_CL_CONTEXT_PLATFORM                         0x1084

// D_cl_command_queue_info
#define D_CL_QUEUE_CONTEXT                            0x1090
#define D_CL_QUEUE_DEVICE                             0x1091
#define D_CL_QUEUE_REFERENCE_COUNT                    0x1092
#define D_CL_QUEUE_PROPERTIES                         0x1093

// D_cl_mem_flags - bitfield
#define D_CL_MEM_READ_WRITE                           (1 << 0)
#define D_CL_MEM_WRITE_ONLY                           (1 << 1)
#define D_CL_MEM_READ_ONLY                            (1 << 2)
#define D_CL_MEM_USE_HOST_PTR                         (1 << 3)
#define D_CL_MEM_ALLOC_HOST_PTR                       (1 << 4)
#define D_CL_MEM_COPY_HOST_PTR                        (1 << 5)

// D_cl_channel_order
#define D_CL_R                                        0x10B0
#define D_CL_A                                        0x10B1
#define D_CL_RG                                       0x10B2
#define D_CL_RA                                       0x10B3
#define D_CL_RGB                                      0x10B4
#define D_CL_RGBA                                     0x10B5
#define D_CL_BGRA                                     0x10B6
#define D_CL_ARGB                                     0x10B7
#define D_CL_INTENSITY                                0x10B8
#define D_CL_LUMINANCE                                0x10B9

// D_cl_channel_type
#define D_CL_SNORM_INT8                               0x10D0
#define D_CL_SNORM_INT16                              0x10D1
#define D_CL_UNORM_INT8                               0x10D2
#define D_CL_UNORM_INT16                              0x10D3
#define D_CL_UNORM_SHORT_565                          0x10D4
#define D_CL_UNORM_SHORT_555                          0x10D5
#define D_CL_UNORM_INT_101010                         0x10D6
#define D_CL_SIGNED_INT8                              0x10D7
#define D_CL_SIGNED_INT16                             0x10D8
#define D_CL_SIGNED_INT32                             0x10D9
#define D_CL_UNSIGNED_INT8                            0x10DA
#define D_CL_UNSIGNED_INT16                           0x10DB
#define D_CL_UNSIGNED_INT32                           0x10DC
#define D_CL_HALF_FLOAT                               0x10DD
#define D_CL_FLOAT                                    0x10DE

// D_cl_mem_object_type
#define D_CL_MEM_OBJECT_BUFFER                        0x10F0
#define D_CL_MEM_OBJECT_IMAGE2D                       0x10F1
#define D_CL_MEM_OBJECT_IMAGE3D                       0x10F2

// D_cl_mem_info
#define D_CL_MEM_TYPE                                 0x1100
#define D_CL_MEM_FLAGS                                0x1101
#define D_CL_MEM_SIZE                                 0x1102
#define D_CL_MEM_HOST_PTR                             0x1103
#define D_CL_MEM_MAP_COUNT                            0x1104
#define D_CL_MEM_REFERENCE_COUNT                      0x1105
#define D_CL_MEM_CONTEXT                              0x1106

// D_cl_image_info
#define D_CL_IMAGE_FORMAT                             0x1110
#define D_CL_IMAGE_ELEMENT_SIZE                       0x1111
#define D_CL_IMAGE_ROW_PITCH                          0x1112
#define D_CL_IMAGE_SLICE_PITCH                        0x1113
#define D_CL_IMAGE_WIDTH                              0x1114
#define D_CL_IMAGE_HEIGHT                             0x1115
#define D_CL_IMAGE_DEPTH                              0x1116

// D_cl_addressing_mode
#define D_CL_ADDRESS_NONE                             0x1130
#define D_CL_ADDRESS_CLAMP_TO_EDGE                    0x1131
#define D_CL_ADDRESS_CLAMP                            0x1132
#define D_CL_ADDRESS_REPEAT                           0x1133

// D_cl_filter_mode
#define D_CL_FILTER_NEAREST                           0x1140
#define D_CL_FILTER_LINEAR                            0x1141

// D_cl_sampler_info
#define D_CL_SAMPLER_REFERENCE_COUNT                  0x1150
#define D_CL_SAMPLER_CONTEXT                          0x1151
#define D_CL_SAMPLER_NORMALIZED_COORDS                0x1152
#define D_CL_SAMPLER_ADDRESSING_MODE                  0x1153
#define D_CL_SAMPLER_FILTER_MODE                      0x1154

// D_cl_map_flags - bitfield
#define D_CL_MAP_READ                                 (1 << 0)
#define D_CL_MAP_WRITE                                (1 << 1)

// D_cl_program_info
#define D_CL_PROGRAM_REFERENCE_COUNT                  0x1160
#define D_CL_PROGRAM_CONTEXT                          0x1161
#define D_CL_PROGRAM_NUM_DEVICES                      0x1162
#define D_CL_PROGRAM_DEVICES                          0x1163
#define D_CL_PROGRAM_SOURCE                           0x1164
#define D_CL_PROGRAM_BINARY_SIZES                     0x1165
#define D_CL_PROGRAM_BINARIES                         0x1166

// D_cl_program_build_info
#define D_CL_PROGRAM_BUILD_STATUS                     0x1181
#define D_CL_PROGRAM_BUILD_OPTIONS                    0x1182
#define D_CL_PROGRAM_BUILD_LOG                        0x1183

// D_cl_build_status
#define D_CL_BUILD_SUCCESS                            0
#define D_CL_BUILD_NONE                               -1
#define D_CL_BUILD_ERROR                              -2
#define D_CL_BUILD_IN_PROGRESS                        -3

// D_cl_kernel_info
#define D_CL_KERNEL_FUNCTION_NAME                     0x1190
#define D_CL_KERNEL_NUM_ARGS                          0x1191
#define D_CL_KERNEL_REFERENCE_COUNT                   0x1192
#define D_CL_KERNEL_CONTEXT                           0x1193
#define D_CL_KERNEL_PROGRAM                           0x1194

// D_cl_kernel_work_group_info
#define D_CL_KERNEL_WORK_GROUP_SIZE                   0x11B0
#define D_CL_KERNEL_COMPILE_WORK_GROUP_SIZE           0x11B1
#define D_CL_KERNEL_LOCAL_MEM_SIZE                    0x11B2

// D_cl_event_info
#define D_CL_EVENT_COMMAND_QUEUE                      0x11D0
#define D_CL_EVENT_COMMAND_TYPE                       0x11D1
#define D_CL_EVENT_REFERENCE_COUNT                    0x11D2
#define D_CL_EVENT_COMMAND_EXECUTION_STATUS           0x11D3

// D_cl_command_type
#define D_CL_COMMAND_NDRANGE_KERNEL                   0x11F0
#define D_CL_COMMAND_TASK                             0x11F1
#define D_CL_COMMAND_NATIVE_KERNEL                    0x11F2
#define D_CL_COMMAND_READ_BUFFER                      0x11F3
#define D_CL_COMMAND_WRITE_BUFFER                     0x11F4
#define D_CL_COMMAND_COPY_BUFFER                      0x11F5
#define D_CL_COMMAND_READ_IMAGE                       0x11F6
#define D_CL_COMMAND_WRITE_IMAGE                      0x11F7
#define D_CL_COMMAND_COPY_IMAGE                       0x11F8
#define D_CL_COMMAND_COPY_IMAGE_TO_BUFFER             0x11F9
#define D_CL_COMMAND_COPY_BUFFER_TO_IMAGE             0x11FA
#define D_CL_COMMAND_MAP_BUFFER                       0x11FB
#define D_CL_COMMAND_MAP_IMAGE                        0x11FC
#define D_CL_COMMAND_UNMAP_MEM_OBJECT                 0x11FD
#define D_CL_COMMAND_MARKER                           0x11FE
#define D_CL_COMMAND_WAIT_FOR_EVENTS                  0x11FF
#define D_CL_COMMAND_BARRIER                          0x1200
#define D_CL_COMMAND_ACQUIRE_GL_OBJECTS               0x1201
#define D_CL_COMMAND_RELEASE_GL_OBJECTS               0x1202

// command execution status
#define D_CL_COMPLETE                                 0x0
#define D_CL_RUNNING                                  0x1
#define D_CL_SUBMITTED                                0x2
#define D_CL_QUEUED                                   0x3
  
// D_cl_profiling_info
#define D_CL_PROFILING_COMMAND_QUEUED                 0x1280
#define D_CL_PROFILING_COMMAND_SUBMIT                 0x1281
#define D_CL_PROFILING_COMMAND_START                  0x1282
#define D_CL_PROFILING_COMMAND_END                    0x1283

/********************************************************************************************************/

// Platform API
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetPlatformIDs(cl_uint          /* num_entries */,
                 D_cl_platform_id * /* platforms */,
                 cl_uint *        /* num_platforms */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL 
clGetPlatformInfo(D_cl_platform_id   /* platform */, 
                  D_cl_platform_info /* param_name */,
                  size_t           /* param_value_size */, 
                  void *           /* param_value */,
                  size_t *         /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

// Device APIs
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetDeviceIDs(D_cl_platform_id   /* platform */,
               D_cl_device_type   /* device_type */, 
               cl_uint          /* num_entries */, 
               D_cl_device_id *   /* devices */, 
               cl_uint *        /* num_devices */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetDeviceInfo(D_cl_device_id    /* device */,
                D_cl_device_info  /* param_name */, 
                size_t          /* param_value_size */, 
                void *          /* param_value */,
                size_t *        /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

// Context APIs  
extern D_CL_API_ENTRY D_cl_context D_CL_API_CALL
clCreateContext(D_cl_context_properties * /* properties */,
                cl_uint                 /* num_devices */,
                const D_cl_device_id *    /* devices */,
                void (*pfn_notify)(const char *, const void *, size_t, void *) /* pfn_notify */,
                void *                  /* user_data */,
                cl_int *                /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY D_cl_context D_CL_API_CALL
clCreateContextFromType(D_cl_context_properties * /* properties */,
                        D_cl_device_type          /* device_type */,
                        void (*pfn_notify)(const char *, const void *, size_t, void *) /* pfn_notify */,
                        void *                  /* user_data */,
                        cl_int *                /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clRetainContext(D_cl_context /* context */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clReleaseContext(D_cl_context /* context */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetContextInfo(D_cl_context         /* context */, 
                 D_cl_context_info    /* param_name */, 
                 size_t             /* param_value_size */, 
                 void *             /* param_value */, 
                 size_t *           /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

// Command Queue APIs
extern D_CL_API_ENTRY D_cl_command_queue D_CL_API_CALL
clCreateCommandQueue(D_cl_context                     /* context */, 
                     D_cl_device_id                   /* device */, 
                     D_cl_command_queue_properties    /* properties */,
                     cl_int *                       /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clRetainCommandQueue(D_cl_command_queue /* command_queue */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clReleaseCommandQueue(D_cl_command_queue /* command_queue */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetCommandQueueInfo(D_cl_command_queue      /* command_queue */,
                      D_cl_command_queue_info /* param_name */,
                      size_t                /* param_value_size */,
                      void *                /* param_value */,
                      size_t *              /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clSetCommandQueueProperty(D_cl_command_queue              /* command_queue */,
                          D_cl_command_queue_properties   /* properties */, 
                          D_cl_bool                        /* enable */,
                          D_cl_command_queue_properties * /* old_properties */) D_CL_API_SUFFIX__VERSION_1_0;

// Memory Object APIs
extern D_CL_API_ENTRY D_cl_mem D_CL_API_CALL
clCreateBuffer(D_cl_context   /* context */,
               D_cl_mem_flags /* flags */,
               size_t       /* size */,
               void *       /* host_ptr */,
               cl_int *     /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY D_cl_mem D_CL_API_CALL
clCreateImage2D(D_cl_context              /* context */,
                D_cl_mem_flags            /* flags */,
                const D_cl_image_format * /* image_format */,
                size_t                  /* image_width */,
                size_t                  /* image_height */,
                size_t                  /* image_row_pitch */, 
                void *                  /* host_ptr */,
                cl_int *                /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;
                        
extern D_CL_API_ENTRY D_cl_mem D_CL_API_CALL
clCreateImage3D(D_cl_context              /* context */,
                D_cl_mem_flags            /* flags */,
                const D_cl_image_format * /* image_format */,
                size_t                  /* image_width */, 
                size_t                  /* image_height */,
                size_t                  /* image_depth */, 
                size_t                  /* image_row_pitch */, 
                size_t                  /* image_slice_pitch */, 
                void *                  /* host_ptr */,
                cl_int *                /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;
                        
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clRetainMemObject(D_cl_mem /* memobj */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clReleaseMemObject(D_cl_mem /* memobj */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetSupportedImageFormats(D_cl_context           /* context */,
                           D_cl_mem_flags         /* flags */,
                           D_cl_mem_object_type   /* image_type */,
                           cl_uint              /* num_entries */,
                           D_cl_image_format *    /* image_formats */,
                           cl_uint *            /* num_image_formats */) D_CL_API_SUFFIX__VERSION_1_0;
                                    
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetMemObjectInfo(D_cl_mem           /* memobj */,
                   D_cl_mem_info      /* param_name */, 
                   size_t           /* param_value_size */,
                   void *           /* param_value */,
                   size_t *         /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetImageInfo(D_cl_mem           /* image */,
               D_cl_image_info    /* param_name */, 
               size_t           /* param_value_size */,
               void *           /* param_value */,
               size_t *         /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

// Sampler APIs
extern D_CL_API_ENTRY D_cl_sampler D_CL_API_CALL
clCreateSampler(D_cl_context          /* context */,
                D_cl_bool             /* normalized_coords */, 
                D_cl_addressing_mode  /* addressing_mode */, 
                D_cl_filter_mode      /* filter_mode */,
                cl_int *            /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clRetainSampler(D_cl_sampler /* sampler */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clReleaseSampler(D_cl_sampler /* sampler */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetSamplerInfo(D_cl_sampler         /* sampler */,
                 D_cl_sampler_info    /* param_name */,
                 size_t             /* param_value_size */,
                 void *             /* param_value */,
                 size_t *           /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;
                            
// Program Object APIs
extern D_CL_API_ENTRY D_cl_program D_CL_API_CALL
clCreateProgramWithSource(D_cl_context        /* context */,
                          cl_uint           /* count */,
                          const char **     /* strings */,
                          const size_t *    /* lengths */,
                          cl_int *          /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY D_cl_program D_CL_API_CALL
clCreateProgramWithBinary(D_cl_context                     /* context */,
                          cl_uint                        /* num_devices */,
                          const D_cl_device_id *           /* device_list */,
                          const size_t *                 /* lengths */,
                          const unsigned char **         /* binaries */,
                          cl_int *                       /* binary_status */,
                          cl_int *                       /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clRetainProgram(D_cl_program /* program */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clReleaseProgram(D_cl_program /* program */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clBuildProgram(D_cl_program           /* program */,
               cl_uint              /* num_devices */,
               const D_cl_device_id * /* device_list */,
               const char *         /* options */, 
               void (*pfn_notify)(D_cl_program /* program */, void * /* user_data */),
               void *               /* user_data */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clUnloadCompiler(void) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetProgramInfo(D_cl_program         /* program */,
                 D_cl_program_info    /* param_name */,
                 size_t             /* param_value_size */,
                 void *             /* param_value */,
                 size_t *           /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetProgramBuildInfo(D_cl_program            /* program */,
                      D_cl_device_id          /* device */,
                      D_cl_program_build_info /* param_name */,
                      size_t                /* param_value_size */,
                      void *                /* param_value */,
                      size_t *              /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;
                            
// Kernel Object APIs
extern D_CL_API_ENTRY D_cl_kernel D_CL_API_CALL
clCreateKernel(D_cl_program      /* program */,
               const char *    /* kernel_name */,
               cl_int *        /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clCreateKernelsInProgram(D_cl_program     /* program */,
                         cl_uint        /* num_kernels */,
                         D_cl_kernel *    /* kernels */,
                         cl_uint *      /* num_kernels_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clRetainKernel(D_cl_kernel    /* kernel */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clReleaseKernel(D_cl_kernel   /* kernel */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clSetKernelArg(D_cl_kernel    /* kernel */,
               cl_uint      /* arg_index */,
               size_t       /* arg_size */,
               const void * /* arg_value */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetKernelInfo(D_cl_kernel       /* kernel */,
                D_cl_kernel_info  /* param_name */,
                size_t          /* param_value_size */,
                void *          /* param_value */,
                size_t *        /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetKernelWorkGroupInfo(D_cl_kernel                  /* kernel */,
                         D_cl_device_id               /* device */,
                         D_cl_kernel_work_group_info  /* param_name */,
                         size_t                     /* param_value_size */,
                         void *                     /* param_value */,
                         size_t *                   /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

// Event Object APIs
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clWaitForEvents(cl_uint             /* num_events */,
                const D_cl_event *    /* event_list */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetEventInfo(D_cl_event         /* event */,
               D_cl_event_info    /* param_name */,
               size_t           /* param_value_size */,
               void *           /* param_value */,
               size_t *         /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;
                            
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clRetainEvent(D_cl_event /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clReleaseEvent(D_cl_event /* event */) D_CL_API_SUFFIX__VERSION_1_0;

// Profiling APIs
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetEventProfilingInfo(D_cl_event            /* event */,
                        D_cl_profiling_info   /* param_name */,
                        size_t              /* param_value_size */,
                        void *              /* param_value */,
                        size_t *            /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;
                                
// Flush D_and Finish APIs
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clFlush(D_cl_command_queue /* command_queue */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clFinish(D_cl_command_queue /* command_queue */) D_CL_API_SUFFIX__VERSION_1_0;

// Enqueued Commands APIs
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueReadBuffer(D_cl_command_queue    /* command_queue */,
                    D_cl_mem              /* buffer */,
                    D_cl_bool             /* blocking_read */,
                    size_t              /* offset */,
                    size_t              /* cb */, 
                    void *              /* ptr */,
                    cl_uint             /* num_events_in_wait_list */,
                    const D_cl_event *    /* event_wait_list */,
                    D_cl_event *          /* event */) D_CL_API_SUFFIX__VERSION_1_0;
                            
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueWriteBuffer(D_cl_command_queue   /* command_queue */, 
                     D_cl_mem             /* buffer */, 
                     D_cl_bool            /* blocking_write */, 
                     size_t             /* offset */, 
                     size_t             /* cb */, 
                     const void *       /* ptr */, 
                     cl_uint            /* num_events_in_wait_list */, 
                     const D_cl_event *   /* event_wait_list */, 
                     D_cl_event *         /* event */) D_CL_API_SUFFIX__VERSION_1_0;
                            
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueCopyBuffer(D_cl_command_queue    /* command_queue */, 
                    D_cl_mem              /* src_buffer */,
                    D_cl_mem              /* dst_buffer */, 
                    size_t              /* src_offset */,
                    size_t              /* dst_offset */,
                    size_t              /* cb */, 
                    cl_uint             /* num_events_in_wait_list */,
                    const D_cl_event *    /* event_wait_list */,
                    D_cl_event *          /* event */) D_CL_API_SUFFIX__VERSION_1_0;
                            
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueReadImage(D_cl_command_queue     /* command_queue */,
                   D_cl_mem               /* image */,
                   D_cl_bool              /* blocking_read */, 
                   const size_t *       /* origin[3] */,
                   const size_t *       /* region[3] */,
                   size_t               /* row_pitch */,
                   size_t               /* slice_pitch */, 
                   void *               /* ptr */,
                   cl_uint              /* num_events_in_wait_list */,
                   const D_cl_event *     /* event_wait_list */,
                   D_cl_event *           /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueWriteImage(D_cl_command_queue    /* command_queue */,
                    D_cl_mem              /* image */,
                    D_cl_bool             /* blocking_write */, 
                    const size_t *      /* origin[3] */,
                    const size_t *      /* region[3] */,
                    size_t              /* input_row_pitch */,
                    size_t              /* input_slice_pitch */, 
                    const void *        /* ptr */,
                    cl_uint             /* num_events_in_wait_list */,
                    const D_cl_event *    /* event_wait_list */,
                    D_cl_event *          /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueCopyImage(D_cl_command_queue     /* command_queue */,
                   D_cl_mem               /* src_image */,
                   D_cl_mem               /* dst_image */, 
                   const size_t *       /* src_origin[3] */,
                   const size_t *       /* dst_origin[3] */,
                   const size_t *       /* region[3] */, 
                   cl_uint              /* num_events_in_wait_list */,
                   const D_cl_event *     /* event_wait_list */,
                   D_cl_event *           /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueCopyImageToBuffer(D_cl_command_queue /* command_queue */,
                           D_cl_mem           /* src_image */,
                           D_cl_mem           /* dst_buffer */, 
                           const size_t *   /* src_origin[3] */,
                           const size_t *   /* region[3] */, 
                           size_t           /* dst_offset */,
                           cl_uint          /* num_events_in_wait_list */,
                           const D_cl_event * /* event_wait_list */,
                           D_cl_event *       /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueCopyBufferToImage(D_cl_command_queue /* command_queue */,
                           D_cl_mem           /* src_buffer */,
                           D_cl_mem           /* dst_image */, 
                           size_t           /* src_offset */,
                           const size_t *   /* dst_origin[3] */,
                           const size_t *   /* region[3] */, 
                           cl_uint          /* num_events_in_wait_list */,
                           const D_cl_event * /* event_wait_list */,
                           D_cl_event *       /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY void * D_CL_API_CALL
clEnqueueMapBuffer(D_cl_command_queue /* command_queue */,
                   D_cl_mem           /* buffer */,
                   D_cl_bool          /* blocking_map */, 
                   D_cl_map_flags     /* map_flags */,
                   size_t           /* offset */,
                   size_t           /* cb */,
                   cl_uint          /* num_events_in_wait_list */,
                   const D_cl_event * /* event_wait_list */,
                   D_cl_event *       /* event */,
                   cl_int *         /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY void * D_CL_API_CALL
clEnqueueMapImage(D_cl_command_queue  /* command_queue */,
                  D_cl_mem            /* image */, 
                  D_cl_bool           /* blocking_map */, 
                  D_cl_map_flags      /* map_flags */, 
                  const size_t *    /* origin[3] */,
                  const size_t *    /* region[3] */,
                  size_t *          /* image_row_pitch */,
                  size_t *          /* image_slice_pitch */,
                  cl_uint           /* num_events_in_wait_list */,
                  const D_cl_event *  /* event_wait_list */,
                  D_cl_event *        /* event */,
                  cl_int *          /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueUnmapMemObject(D_cl_command_queue /* command_queue */,
                        D_cl_mem           /* memobj */,
                        void *           /* mapped_ptr */,
                        cl_uint          /* num_events_in_wait_list */,
                        const D_cl_event *  /* event_wait_list */,
                        D_cl_event *        /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueNDRangeKernel(D_cl_command_queue /* command_queue */,
                       D_cl_kernel        /* kernel */,
                       cl_uint          /* work_dim */,
                       const size_t *   /* global_work_offset */,
                       const size_t *   /* global_work_size */,
                       const size_t *   /* local_work_size */,
                       cl_uint          /* num_events_in_wait_list */,
                       const D_cl_event * /* event_wait_list */,
                       D_cl_event *       /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueTask(D_cl_command_queue  /* command_queue */,
              D_cl_kernel         /* kernel */,
              cl_uint           /* num_events_in_wait_list */,
              const D_cl_event *  /* event_wait_list */,
              D_cl_event *        /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueNativeKernel(D_cl_command_queue  /* command_queue */,
					  void (*user_func)(void *), 
                      void *            /* args */,
                      size_t            /* cb_args */, 
                      cl_uint           /* num_mem_objects */,
                      const D_cl_mem *    /* mem_list */,
                      const void **     /* args_mem_loc */,
                      cl_uint           /* num_events_in_wait_list */,
                      const D_cl_event *  /* event_wait_list */,
                      D_cl_event *        /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueMarker(D_cl_command_queue    /* command_queue */,
                D_cl_event *          /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueWaitForEvents(D_cl_command_queue /* command_queue */,
                       cl_uint          /* num_events */,
                       const D_cl_event * /* event_list */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueBarrier(D_cl_command_queue /* command_queue */) D_CL_API_SUFFIX__VERSION_1_0;

#ifdef __cplusplus
}
#endif

#endif  // D___OPENCL_CL_H

