/**********************************************************************************
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
 **********************************************************************************/

#ifndef D___OPENCL_CL_GL_H
#define D___OPENCL_CL_GL_H

#ifdef __APPLE__
#include <OpenCL/cl_platform.h>
#else
#include <CL/cl_platform.h>
#endif	

#ifdef __cplusplus
extern "C" {
#endif

// NOTE:  Make sure that appropriate GL header file D_is included separately

typedef cl_uint     D_cl_gl_object_type;
typedef cl_uint     D_cl_gl_texture_info;
typedef cl_uint     D_cl_gl_platform_info;

// D_cl_gl_object_type
#define D_CL_GL_OBJECT_BUFFER             0x2000
#define D_CL_GL_OBJECT_TEXTURE2D          0x2001
#define D_CL_GL_OBJECT_TEXTURE3D          0x2002
#define D_CL_GL_OBJECT_RENDERBUFFER       0x2003

// D_cl_gl_texture_info
#define D_CL_GL_TEXTURE_TARGET            0x2004
#define D_CL_GL_MIPMAP_LEVEL              0x2005

extern D_CL_API_ENTRY D_cl_mem D_CL_API_CALL
clCreateFromGLBuffer(D_cl_context     /* context */,
                     D_cl_mem_flags   /* flags */,
                     GLuint         /* bufobj */,
                     int *          /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY D_cl_mem D_CL_API_CALL
clCreateFromGLTexture2D(D_cl_context      /* context */,
                        D_cl_mem_flags    /* flags */,
                        GLenum          /* target */,
                        GLint           /* miplevel */,
                        GLuint          /* texture */,
                        cl_int *        /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY D_cl_mem D_CL_API_CALL
clCreateFromGLTexture3D(D_cl_context      /* context */,
                        D_cl_mem_flags    /* flags */,
                        GLenum          /* target */,
                        GLint           /* miplevel */,
                        GLuint          /* texture */,
                        cl_int *        /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY D_cl_mem D_CL_API_CALL
clCreateFromGLRenderbuffer(D_cl_context   /* context */,
                           D_cl_mem_flags /* flags */,
                           GLuint       /* renderbuffer */,
                           cl_int *     /* errcode_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetGLObjectInfo(D_cl_mem                /* memobj */,
                  D_cl_gl_object_type *   /* gl_object_type */,
                  GLuint *              /* gl_object_name */) D_CL_API_SUFFIX__VERSION_1_0;
                  
extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clGetGLTextureInfo(D_cl_mem               /* memobj */,
                   D_cl_gl_texture_info   /* param_name */,
                   size_t               /* param_value_size */,
                   void *               /* param_value */,
                   size_t *             /* param_value_size_ret */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueAcquireGLObjects(D_cl_command_queue      /* command_queue */,
                          cl_uint               /* num_objects */,
                          const D_cl_mem *        /* mem_objects */,
                          cl_uint               /* num_events_in_wait_list */,
                          const D_cl_event *      /* event_wait_list */,
                          D_cl_event *            /* event */) D_CL_API_SUFFIX__VERSION_1_0;

extern D_CL_API_ENTRY cl_int D_CL_API_CALL
clEnqueueReleaseGLObjects(D_cl_command_queue      /* command_queue */,
                          cl_uint               /* num_objects */,
                          const D_cl_mem *        /* mem_objects */,
                          cl_uint               /* num_events_in_wait_list */,
                          const D_cl_event *      /* event_wait_list */,
                          D_cl_event *            /* event */) D_CL_API_SUFFIX__VERSION_1_0;

#ifdef __cplusplus
}
#endif

#endif  // D___OPENCL_CL_GL_H
