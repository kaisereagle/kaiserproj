//
//  ViewController.m
//  base
//
//  Created by kaiser on 2018/01/23.
//  Copyright © 2018年 kaiser. All rights reserved.
//

#import "ViewController.h"
#include "common.h"
#include "GLMetaseq.h"
#include "core.h"
#include <OpenGLES/ES1/gl.h>
#include <OpenGLES/ES1/glext.h>
#import     <OpenGLES/ES2/gl.h>
#import     <OpenGLES/ES2/glext.h>
//#include "PMDModel.h"
//#include "VMDMotion.h"
//#include    "BulletPhysics.h"
//static cPMDModel g_clPMDModel;
//static cVMDMotion g_clVMDMotion;
extern "C"{
#include "data.h"

}
#import "MMDAgent.h"
PMDObject *m_models;
MMDAgent *mmdagent;
PMDModel *m_pmd;
// グローバル変数
int g_mouseX = 0;    // マウスX座標
int g_mouseY = 0;    // マウスY座標
int g_rotX = 0;        // X軸周りの回転
int    g_rotY = 0;        // Y軸周りの回転

MQO_MODEL g_mqoModel;    // MQOモデル

// 光源の設定を行う関数
void mySetLight(void)
{
    GLfloat light_diffuse[]  = { 0.9, 0.9, 0.9, 1.0 };    // 拡散反射光
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };    // 鏡面反射光
    GLfloat light_ambient[]  = { 0.3, 0.3, 0.3, 0.1 };    // 環境光
    GLfloat light_position[] = { 0.0, 0.0, 0.0, 1.0 };    // 位置と種類
    
    // 光源の設定
    glLightfv( GL_LIGHT0, GL_DIFFUSE,  light_diffuse );     // 拡散反射光の設定
    glLightfv( GL_LIGHT0, GL_SPECULAR, light_specular ); // 鏡面反射光の設定
    glLightfv( GL_LIGHT0, GL_AMBIENT,  light_ambient );     // 環境光の設定
    glLightfv( GL_LIGHT0, GL_POSITION, light_position ); // 位置と種類の設定
    
    glShadeModel( GL_SMOOTH );    // シェーディングの種類の設定
    glEnable( GL_LIGHT0 );        // 光源の有効化
}

// 描画関数
void Draw()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    // 初期化
    glClearColor(0.f, 0.f, 0.2f, 1.0);                    // 背景色
    glMatrixMode(GL_MODELVIEW);
    
    glEnable( GL_DEPTH_TEST );        // 隠面処理の適用
    mySetLight();                    // 光源の設定
    glEnable( GL_LIGHTING );        // 光源ON
    
    glPushMatrix();
    glTranslatef(0.0, -100.0, -400.0);    // 平行移動
    glRotatef( g_rotX, 1, 0, 0 );        // X軸回転
    glRotatef( g_rotY, 0, 1, 0 );        // Y軸回転
    mqoCallModel(g_mqoModel);            // MQOモデルのコール
    glPopMatrix();
    
    glDisable( GL_LIGHTING );
    glDisable( GL_DEPTH_TEST );
    //glutSwapBuffers();
}
class gl_mac :public nammu::core
{
//    CIM3DPrimitive * _prim;
  //  CIM3DShader * _shader;
    
    //ビュー変換行列
    //_X3DMATRIX m_modelViewProjectionMatrix;
    //法線の変換行列
    //_X3DMATRIX3 m_normalMatrix;
    //オブジェクトの回転角度
    float m_rotation;
    // カラーバッファ
    GLuint   buffer_color;
    // 深度＆ステンシルバッファ
    GLuint   buffer_depth_stencil;
    // レンダリング用のContext
    EAGLContext *renderingContext;
public:
    GLuint vertexBufferID;
    void init()
    {
        //renderingContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
        glEnable(GL_DEPTH_TEST);
        glFrontFace(GL_CCW);
        glCullFace(GL_BACK);
        glEnable(GL_CULL_FACE);
        
    }
    void draw()
    {
        //glGenRenderbuffers(1, &buffer_color);
        //glClearColor(1.65f, 0.65f, 0.65f, 1.0f);
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        Draw();
        //glBindRenderbuffer(GL_RENDERBUFFER, buffer_color);
        [renderingContext presentRenderbuffer:GL_RENDERBUFFER];
        
    }
    
    
};
gl_mac _mac_core;
typedef struct {
    GLKVector3 position;
} Vertex;
//　三角の座標
// verticesはvertex（頂点）の複数形
static const Vertex vertices[] =
{
    {{-0.5f, -0.5f,  0.0}},
    {{ 0.5f, -0.5f,  0.0}},
    {{-0.5f,  0.5f,  0.0}}
};
/**
 * シェーダーの読み込みを行う。
 */
static GLint Shader_load(const char* shader_source, const GLenum GL_XXXX_SHADER) {
    const GLint shader = glCreateShader(GL_XXXX_SHADER);
    assert(glGetError() == GL_NO_ERROR);
    
    glShaderSource(shader, 1, &shader_source, NULL);
    glCompileShader(shader);
    
    // コンパイルエラーをチェックする
    {
        GLint compileSuccess = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compileSuccess);
        if (compileSuccess == GL_FALSE) {
            // エラーが発生した
            GLint infoLen = 0;
            // エラーメッセージを取得
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
            if (infoLen > 1) {
                GLchar *message = (GLchar*) calloc(infoLen, sizeof(GLchar));
                glGetShaderInfoLog(shader, infoLen, NULL, message);
                //__log(message);
                free((void*) message);
            } else {
                __log("comple error not info...");
            }
        }
        
        // コンパイル失敗していたらここでプログラムを停止する
        assert(compileSuccess == GL_TRUE);
    }
    
    return shader;
}

/**
 * 頂点・フラグメントシェーダーをリンクし、ShaderProgramを作成する
 */
GLuint Shader_createProgramFromSource(const char* vertex_shader_source, const char* fragment_shader_source) {
    const GLuint vertex_shader = Shader_load(vertex_shader_source, GL_VERTEX_SHADER);
    const GLuint fragment_shader = Shader_load(fragment_shader_source, GL_FRAGMENT_SHADER);
    
    
    const GLuint program = glCreateProgram();
    assert(glGetError() == GL_NO_ERROR);
    assert(program != 0);
    
    glAttachShader(program, vertex_shader); // バーテックスシェーダーとプログラムを関連付ける
    glAttachShader(program, fragment_shader); // フラグメントシェーダーとプログラムを関連付ける
    assert(glGetError() == GL_NO_ERROR);
    
    // コンパイルを行う
    glLinkProgram(program);
    
    // リンクエラーをチェックする
    {
        GLint linkSuccess = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &linkSuccess);
        if (linkSuccess == GL_FALSE) {
            // エラーが発生した
            GLint infoLen = 0;
            // エラーメッセージを取得
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLen);
            if (infoLen > 1) {
                GLchar *message = (GLchar*) calloc(infoLen, sizeof(GLchar));
                glGetProgramInfoLog(program, infoLen, NULL, message);
//                __log(message);
                free((void*) message);
            }
        }
        assert(linkSuccess == GL_TRUE);
    }
    
    // リンク済みのため、個々のシェーダーオブジェクトの解放フラグを立てる
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    
    // リンク済みのプログラムを返す
    return program;
}

typedef struct {
    // レンダリング用シェーダープログラム
    GLuint shader_program;
    
    // 位置情報属性
    GLint attr_pos;
    
    // フラグメントシェーダの描画色
    GLint unif_color;
    
} Extension_BlendOrder;
//Extension_BlendOrder *extension;
typedef struct {
    // レンダリング用シェーダープログラム
    GLuint shader_program;
    
    // 位置情報属性
    GLint attr_pos;
    
    // UV座標属性
    GLint attr_uv;
    
    // フラグメントシェーダの描画色
    GLint unif_color;
    
    // Diffuseテクスチャ
    GLint unif_tex_diffuse;
    
    // 描画行列
    GLint unif_wlp;
    
    // サンプル用のPMDファイル
    PmdFile *pmd;
    
    // サンプルPMD用のテクスチャリスト
    PmdTextureList *textureList;
    
    // フィギュアの回転
    GLfloat rotate;
} Extension_PmdLoad;
Extension_PmdLoad *extension;
GLApplication *app;
void * getbin(const char *file_name,int *in_size)
{
    NSString *fileName = [[NSString alloc ] initWithFormat:@"/%@", CSTR2NSSTRING(file_name)];
    NSBundle *bundle  = [NSBundle mainBundle];
    fileName = [bundle pathForResource:fileName ofType:@""];
    NSData *data = [NSData dataWithContentsOfMappedFile:fileName];
    
    if(!data) {
        __logf("file not found(%s)", file_name);
        return NULL;
    }
    int size =(int)data.length;
    void *bindata= (void*)malloc(sizeof(size));
    *in_size = size;
    
    return bindata;
    
}
//extern cBulletPhysics g_clBulletPhysics;


@interface ViewController ()
{}
@end

@implementation ViewController


- (void)viewDidLoad {
     [super viewDidLoad];
    m_models=new PMDObject();
    mmdagent=new MMDAgent();
    char *fileName;
    int i;
    int id;
    int baseID;
    char *name;
    btVector3 offsetPos(btScalar(0.0f), btScalar(0.0f), btScalar(0.0f));
    btQuaternion offsetRot(btScalar(0.0f), btScalar(0.0f), btScalar(0.0f), btScalar(1.0f));
    bool forcedPosition = false;
    PMDBone *assignBone = NULL;
    PMDObject *assignObject = NULL;
    SystemTexture *m_systex = new SystemTexture();
    m_pmd=new PMDModel(mmdagent);
    m_pmd->loadfile(nil, nil);
    
    // スケールファクタを設定する
    // Retinaディスプレイの場合でも正しい解像度でレンダリングバッファを作成するようになる。
   // self.contentScaleFactor = [UIScreen mainScreen].scale;
    app=(GLApplication*)calloc(1, sizeof(GLApplication));

    app->extension = (Extension_PmdLoad*) malloc(sizeof(Extension_PmdLoad));
    // ES2.0用にContextを作成する。
    // ContextはOpenGL ESが適用したステートを保存し、OpenGL ESはこのコンテキストに従って動作を行う。
    renderingContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    extension = (Extension_PmdLoad*) app->extension;
        const BOOL complete = [renderingContext renderbufferStorage:GL_RENDERBUFFER fromDrawable:(CAEAGLLayer*)self.view];
    // 初期化スレッドに対して関連付ける
    {
        
        BOOL current = [EAGLContext setCurrentContext:renderingContext];
        assert(current == YES);
    }
 
    // 描画対象のバッファを作成する
    // フレームバッファ作成
    glGenFramebuffers(1, &buffer_frame);
    assert(buffer_frame);
    
    // 色用バッファを作成
    glGenRenderbuffers(1, &buffer_color);
    assert(buffer_color);
    
    // 深度・ステンシル（型抜き）混合バッファを作成
    glGenRenderbuffers(1, &buffer_depth_stencil);
    assert(buffer_depth_stencil);
    
    {
        glFinish();
        BOOL current = [EAGLContext setCurrentContext:nil];
        assert(current == YES);
    }
    [self bind];
    // シェーダーを用意する
    {
        const GLchar *vertex_shader_source =
        // attributes
        "attribute mediump vec4 attr_pos;"
        "attribute mediump vec2 attr_uv;"
        
        // uniforms
        "uniform mediump mat4 unif_wlp;"
        // varyings
        "varying mediump vec2 vary_uv;"
        // main
        "void main() {"
        "   gl_Position = unif_wlp * attr_pos;"
        "   vary_uv = attr_uv;"
        "}";
        
        const GLchar *fragment_shader_source =
        
        // uniforms
        "uniform lowp vec4 unif_color;"
        "uniform sampler2D unif_tex_diffuse;"
        // varyings
        "varying mediump vec2 vary_uv;"
        // main
        "void main() {"
        "   if(unif_color.a == 0.0) {"
        "       gl_FragColor = texture2D(unif_tex_diffuse, vary_uv);"
        "   } else {"
        "       gl_FragColor = unif_color;"
        "   }"
        "}";
        
        // コンパイルとリンクを行う
        extension->shader_program = Shader_createProgramFromSource(vertex_shader_source, fragment_shader_source);
    }
    
    // attributeを取り出す
    {
        extension->attr_pos = glGetAttribLocation(extension->shader_program, "attr_pos");
        assert(extension->attr_pos >= 0);
        
        extension->attr_uv = glGetAttribLocation(extension->shader_program, "attr_uv");
        assert(extension->attr_uv >= 0);
    }
    
    // uniform変数のlocationを取得する
    {
        extension->unif_wlp = glGetUniformLocation(extension->shader_program, "unif_wlp");
        assert(extension->unif_wlp >= 0);
        
        extension->unif_color = glGetUniformLocation(extension->shader_program, "unif_color");
        assert(extension->unif_color >= 0);
        
        extension->unif_tex_diffuse = glGetUniformLocation(extension->shader_program, "unif_tex_diffuse");
        assert(extension->unif_tex_diffuse >= 0);
    }
    
    {
     //   g_clPMDModel.load("./pmd-sample.pmd");              // PMDファイルを登録
        
//       RawData *data = RawData_loadFile(app, "miku.pmd");
//        g_clPMDModel.initialize((unsigned char *)data->read_head,data->length);
//        g_clVMDMotion.load("./1.vmd");          // VMDファイルを登録
//        g_clPMDModel.setMotion(&g_clVMDMotion, false);
//        g_clPMDModel.updateMotion(0.0f);
//         PMDを読み込む
        extension->pmd = PmdFile_load(app, "miku.pmd");
        assert(extension->pmd);
        
        // テクスチャを読み込む
      //  extension->textureList = PmdFile_createTextureList(app, extension->pmd);
        
        // 読み込んだテクスチャファイル名を表示
//        int i = 0;
  //      for (i = 0; i < extension->textureList->textures_num; ++i) {
    //        __logf("Mat[%d] name(%s)", i, extension->textureList->texture_names[i]);
      //  }
        
        extension->rotate = 0;
    }
    
    // シェーダーの利用を開始する
    glUseProgram(extension->shader_program);
    assert(glGetError() == GL_NO_ERROR);
    
    // 深度テストを有効にする
    glEnable(GL_DEPTH_TEST);
    GLKView *view = (GLKView *)self.view;
    view.context = renderingContext;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    app->surface_width=640;
    app->surface_height = 1008;
    //[self resizeRenderBuffer];
    //[self postFrontBuffer];
//[view bindDrawable];

    // Do any additional setup after loading the view, typically from a nib.
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

// OpenGL ES 2.0の専有を開始する
- (void)bind {
    BOOL current = [EAGLContext setCurrentContext:renderingContext];
    assert(current == YES);
    glBindFramebuffer(GL_FRAMEBUFFER, buffer_frame);
    assert(glGetError() == GL_NO_ERROR);
}
// OpenGL ES 2.0の専有を終了する
- (void) unbind {
    glFinish();
    BOOL current = [EAGLContext setCurrentContext:nil];
    assert(current == YES);
}

// OpenGL ES 2.0の描画を終了し、画面へレンダリングを反映する
- (void)postFrontBuffer {
    // presentRenderbufferは現在バインドされているレンダーバッファをフロントバッファへ転送する
    glBindRenderbuffer(GL_RENDERBUFFER, buffer_color);
    [renderingContext presentRenderbuffer:GL_RENDERBUFFER];
}

#define BUFFER_OFFSET(bytes) ((GLubyte *)NULL + (bytes))

- (void)update
{

    __log("update func ");
}


- (void)oglkView:(GLKView *)view drawInRect:(CGRect)rect
{
    
    __log("glk func ");
    
}
/**
 * レンダリングバッファのリサイズを行う
 */
- (void) resizeRenderBuffer {
    // 現在バインドされているフレームバッファ名を取得
    GLint currentFrameBufferName = 0;
    glGetIntegerv( GL_FRAMEBUFFER_BINDING, &currentFrameBufferName );
    
    // フレームバッファがバインドされていないと情報が取得できない
  //  if ( currentFrameBufferName == 0 ) { return false; }
    
    // あとで元に戻すために現在バインドされているレンダーバッファ名を取得
    GLint oldRenderBufferName = 0;
    glGetIntegerv( GL_RENDERBUFFER_BINDING, &oldRenderBufferName );
    
    // 現在のフレームバッファにアタッチされているカラーバッファのレンダーバッファ名を取得
    GLint colorBufferName = 0;
    glGetFramebufferAttachmentParameteriv( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &colorBufferName );
    
    // カラーバッファがアタッチされていない？
   // if ( colorBufferName == 0 ) { return false; }
    
    // レンダーバッファ(カラーバッファ)をバインド
    glBindRenderbuffer( GL_RENDERBUFFER, colorBufferName );
    GLint width=640;
    GLint height=640;

    // カラーバッファの幅と高さを取得
    glGetRenderbufferParameteriv( GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &width);
    glGetRenderbufferParameteriv( GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &height );
    
    // レンダーバッファのバインドを元に戻す
    glBindRenderbuffer( GL_RENDERBUFFER, oldRenderBufferName );

}
- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
   // [self.baseEffect prepareToDraw];
    extension = (Extension_PmdLoad*) app->extension;

        __log("glk func ");
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 属性を有効にする
    glEnableVertexAttribArray(extension->attr_pos);
    glEnableVertexAttribArray(extension->attr_uv);
    
    mat4 wlpMatrix;
   // g_clPMDModel.render();
    float m_lightDirection[4];
    m_lightDirection[0] = OPTION_LIGHTDIRECTIONX_DEF;
    m_lightDirection[1] = OPTION_LIGHTDIRECTIONY_DEF;
    m_lightDirection[2] = OPTION_LIGHTDIRECTIONZ_DEF;
    m_lightDirection[3] = OPTION_LIGHTDIRECTIONI_DEF;
    m_pmd->renderModel2(m_lightDirection);
    // カメラを初期化する
//    {
//        vec3 pmdMax;
//        vec3 pmdMin;
//
//        PmdFile_calcAABB(extension->pmd, &pmdMin, &pmdMax);
//
//        // カメラをセットアップする
//        const vec3 camera_pos = vec3_create(0, pmdMax.y * 0.7f, pmdMin.z * 7.0f); // カメラ位置
//        const vec3 camera_look = vec3_create(0, pmdMax.y * 0.3f, 0); // カメラ注視
//        const vec3 camera_up = vec3_create(0, 1, 0); // カメラ上ベクトル
//
//        const GLfloat prj_near = 1.0f;
//        const GLfloat prj_far = (pmdMax.z - pmdMin.z) * 30.0f;
//        const GLfloat prj_fovY = 45.0f;
//        const GLfloat prj_aspect = (GLfloat) (app->surface_width) / (GLfloat) (app->surface_height);
//
//        const mat4 lookMatrix = mat4_lookAt(camera_pos, camera_look, camera_up);
//        const mat4 projectionMatrix = mat4_perspective(prj_near, prj_far, prj_fovY, prj_aspect);
//
//        const mat4 world = mat4_rotate(vec3_create(0, 1, 0), extension->rotate);
//
//        wlpMatrix = mat4_multiply(projectionMatrix, lookMatrix);
//        wlpMatrix = mat4_multiply(wlpMatrix, world);
//
//       // extension->rotate += 1;
//    }
//
//    // PMDのレンダリングを行う
//    {
//        PmdFile *pmd = extension->pmd;
//        int i = 0;
//
//        // 頂点をバインドする
//        glVertexAttribPointer(extension->attr_pos, 3, GL_FLOAT, GL_FALSE, sizeof(PmdVertex), &pmd->vertices[0].position);
//        glVertexAttribPointer(extension->attr_uv, 2, GL_FLOAT, GL_FALSE, sizeof(PmdVertex), &pmd->vertices[0].uv);
//
//        // 描画行列アップロード
//        glUniformMatrix4fv(extension->unif_wlp, 1, GL_FALSE, (GLfloat*) wlpMatrix.m);
//
//        GLint beginIndicesIndex = 0;
//
//        // マテリアル数だけ描画を行う
//        for (i = 0; i < pmd->materials_num; ++i) {
//
//            PmdMaterial *mat = &pmd->materials[i];
//
//            // テクスチャを取り出す
//            Texture *tex = PmdFile_getTexture(extension->textureList, mat->diffuse_texture_name);
//            if (tex) {
//                // テクスチャがロードできている
//                glBindTexture(GL_TEXTURE_2D, tex->id);
//                glUniform1i(extension->unif_tex_diffuse, 0);
//                glUniform4f(extension->unif_color, 0, 0, 0, 0);
//            } else {
//                // カラー情報
//                glUniform4f(extension->unif_color, mat->diffuse.x, mat->diffuse.y, mat->diffuse.z, mat->diffuse.w);
//            }
//
//            // インデックスバッファでレンダリング
//            glDrawElements(GL_TRIANGLES, mat->indices_num, GL_UNSIGNED_SHORT, pmd->indices + beginIndicesIndex);
//            GLenum t=glGetError();
//            //GLKView *view = (GLKView *)self.view;
//           // view.context = renderingContext;
//           // view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
//            assert(glGetError() == GL_NO_ERROR);
//            beginIndicesIndex += mat->indices_num;
//        }
//    }
    [self postFrontBuffer];
    [NSThread sleepForTimeInterval:0.001f];
}
@end
