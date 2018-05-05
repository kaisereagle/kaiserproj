//
//  data.h
//  base
//
//  Created by kaiser on 2018/01/29.
//  Copyright © 2018年 kaiser. All rights reserved.
//

#ifndef data_h
#define data_h

#define GLAPP_FLAG_ABORT (0x1 << 2)
#ifndef _WINDOWS
extern void ES20_sjis2utf8(GLchar *str, const int array_length);
#else

#endif // !_WINDOWS


struct GLApplication;

/**
 * アプリの初期化を行う
 */
typedef void (*GLApplication_initialize)(struct GLApplication *app);

/**
 * アプリの非同期処理を行う
 */
typedef void (*GLApplication_async)(struct GLApplication *app);

/**
 * レンダリングエリアが変更された
 */
typedef void (*GLApplication_surface_changed)(struct GLApplication *app);

/**
 * アプリのレンダリングを行う
 * 毎秒60回前後呼び出される。
 */
typedef void (*GLApplication_rendering)(struct GLApplication *app);

/**
 * アプリのデータ削除を行う
 */
typedef void (*GLApplication_destroy)(struct GLApplication *app);

/**
 * アプリに与えられるデータ
 * プラットフォームごとに適当な拡張を行なっている。
 */
typedef struct GLApplication {
    /**
     * View（レンダリングターゲット）の幅
     */
    int surface_width;
    
    /**
     * View（レンダリングターゲット）の高さ
     */
    int surface_height;
    
    /**
     * アプリ実行フラグ
     */
    int flags;
    
    /**
     * プラットフォーム固有データを格納する。
     * サンプルアプリから扱う必要はない。
     */
    void* platform;
    
    /**
     * 初期化関数
     */
    GLApplication_initialize initialize;
    
    /**
     * 非同期処理
     */
    GLApplication_async async;
    
    /**
     * サーフェイスサイズ変更関数
     */
    GLApplication_surface_changed resized;
    
    /**
     * レンダリング関数
     */
    GLApplication_rendering rendering;
    
    /**
     * 解放関数
     */
    GLApplication_destroy destroy;
    
    /**
     * サンプルごとの拡張データ保持用変数。
     * 自由に構造体等をallocして利用する。
     * XXX_destroy時に必ずfreeを行うこと。
     */
    void* extension;
} GLApplication;

/**
 * ファイルのフルパス -> ファイル名に変換する('/'区切り専用)
 */
extern char* util_getFileName(char* __file__);

/**
 * ダイアログを出して実行を停止する
 */
extern void GLApplication_abortWithMessage(GLApplication *app, const char* message);

/**
 * アプリ実行を停止している場合はapp_TRUE
 */
extern bool GLApplication_isAbort(GLApplication *app);

/**
 * 生ファイル情報を保持する
 */
typedef struct RawData {
    /**
     * データ配列の先頭ポインタ
     */
    void* head;
    
    /**
     * データ配列の長さ（byte）
     */
    int length;
    
    /**
     * 読込中のヘッダ位置
     */
    uint8_t *read_head;
} RawData;

/**
 * assets配下からファイルを読み込む
 */
#ifndef _WINDOWS
extern RawData* RawData_loadFile(GLApplication *app, const char* file_name);
#else
RawData* RawData_loadFile(GLApplication *app, const char* file_name);

#endif
/**
 * 読み込んだファイルを解放する
 */
extern void RawData_freeFile(GLApplication *app, RawData *rawData);

/**
 * データ全体の長さを取得する
 */
extern int RawData_getLength(RawData *rawData);

/**
 * 現在の読み取りポインタ位置を取得する
 */
extern void* RawData_getReadHeader(RawData *rawData);

/**
 * 読み取りポインタを指定バイト数移動させる
 */
extern void RawData_offsetHeader(RawData *rawData, int offsetBytes);

/**
 * 読み込みヘッダの位置を指定位置に移動させる
 */
extern void RawData_setHeaderPosition(RawData *rawData, int position);

/**
 * 読み込める残量を取得する
 */
extern int RawData_getAvailableBytes(RawData *rawData);

/**
 * 指定バイト数の情報を読み込む
 */
extern void RawData_readBytes(RawData* rawData, void* result, int bytes);

/**
 * 8bit整数を読み込む
 */
extern int8_t RawData_read8(RawData* rawData);

/**
 * Big Endian格納の16bit整数を読み込む
 */
extern int16_t RawData_readBE16(RawData* rawData);

/**
 * Big Endian格納の32bit整数を読み込む
 */
extern int32_t RawData_readBE32(RawData* rawData);

/**
 * Little Endian格納の16bit整数を読み込む
 */
extern int16_t RawData_readLE16(RawData* rawData);

/**
 * Little Endian格納の32bit整数を読み込む
 */
extern int32_t RawData_readLE32(RawData* rawData);
/**
 * 360度系からラジアン角度に修正する
 */
#define degree2radian(degree) ((degree * M_PI ) / 180.0)

/**
 * 2次元ベクトルを保持する構造体
 */
typedef struct vec2 {
    /**
     * X値
     */
    GLfloat x;
    
    /**
     * Y値
     */
    GLfloat y;
} vec2;

/**
 * 3次元ベクトルを保持する構造体
 */
typedef struct vec3 {
    /**
     * X値
     */
    GLfloat x;
    
    /**
     * Y値
     */
    GLfloat y;
    
    /**
     * Z値
     */
    GLfloat z;
} vec3;

/**
 * 4次元ベクトルを保持する構造体
 */
typedef struct vec4 {
    /**
     * X値
     */
    GLfloat x;
    
    /**
     * Y値
     */
    GLfloat y;
    
    /**
     * Z値
     */
    GLfloat z;
    
    /**
     * W値
     */
    GLfloat w;
} vec4;

/**
 * 行列を保持する構造体
 */
typedef struct mat4 {
    GLfloat m[4][4];
} mat4;

/**
 * 2次元ベクトルを生成する
 */
extern vec2 vec2_create(const GLfloat x, const GLfloat y);

/**
 * 3次元ベクトルを生成する
 */
extern vec3 vec3_create(const GLfloat x, const GLfloat y, const GLfloat z);

/**
 * 正規化した3次元ベクトルを生成する
 */
extern vec3 vec3_createNormalized(const GLfloat x, const GLfloat y, const GLfloat z);

/**
 * 3次元ベクトルの長さを取得する
 */
extern GLfloat vec3_length(const vec3 v);

/**
 * 3次元ベクトルを正規化する
 */
extern vec3 vec3_normalize(const vec3 v);

/**
 * 3次元ベクトルの内積を計算する
 */
extern GLfloat vec3_dot(const vec3 v0, const vec3 v1);

/**
 * 3次元ベクトルの外積を計算する
 */
extern vec3 vec3_cross(const vec3 v0, const vec3 v1);

/**
 * 単位行列を生成する
 */
extern mat4 mat4_identity();

/**
 * 移動行列を作成する
 */
extern mat4 mat4_translate(const GLfloat x, const GLfloat y, const GLfloat z);

/**
 * 拡縮行列を作成する
 */
extern mat4 mat4_scale(const GLfloat x, const GLfloat y, const GLfloat z);

/**
 * 回転行列を生成する
 */
extern mat4 mat4_rotate(const vec3 axis, const GLfloat rotate);

/**
 * 行列の乗算を行う
 */
extern mat4 mat4_multiply(const mat4 a, const mat4 b);

/**
 * 視点変換行列を生成する
 */
extern mat4 mat4_lookAt(const vec3 eye, const vec3 look, const vec3 up);

/**
 * 射影行列を生成する
 */
extern mat4 mat4_perspective(const GLfloat near, const GLfloat far, const GLfloat fovY_degree, const GLfloat aspect);

/**
 * RGBA各8bit色
 *
 * 32bit / pixel
 */
#define TEXTURE_RAW_RGBA8         0

/**
 * RGB各8bit色
 *
 * 24bit / pixel
 */
#define TEXTURE_RAW_RGB8          1

/**
 * RGB各5bit
 * A 1bit
 *
 * 16bit / pixel
 */
#define TEXTURE_RAW_RGBA5551     2

/**
 * R 5bit
 * G 6bit
 * B 5bit
 *
 * 16bit/ pixel
 */
#define TEXTURE_RAW_RGB565       3

/**
 * 読み込んだ画像のピクセル情報をそのまま保存する構造体
 */
typedef struct RawPixelImage {
    /**
     * ピクセル配列
     */
    void* pixel_data;
    
    /**
     * 画像幅
     */
    int width;
    
    /**
     * 画像高さ
     */
    int height;
    
    /**
     * 画像フォーマット（下記のいずれか）
     * TEXTURE_RAW_RGBA8
     * TEXTURE_RAW_RGB8
     * TEXTURE_RAW_RGBA5551
     * TEXTURE_RAW_RGB565
     */
    int format;
} RawPixelImage;
/**
 * 画像を読み込む。
 * 読み込んだ画像はes20_freeImage()で解放する
 */
#ifndef _WINDOWS

extern RawPixelImage* RawPixelImage_load(GLApplication *app, const char* file_name, const int pixel_format);

#endif // !_WINDOWS


/**
 * es20_loadImage()関数から読み込んだ画像を解放する
 */
extern void RawPixelImage_free(GLApplication *app, RawPixelImage *image);

/**
 * RGB888のポインタをdst_pixelsへピクセル情報をコピーする。
 */
extern void RawPixelImage_convertColorRGB(const void *rgb888_pixels, const int pixel_format, void *dst_pixels, const int pixel_num);

/**
 * RGBA8888のポインタをdst_pixelsへピクセル情報をコピーする。
 */
extern void RawPixelImage_convertColorRGBA(const void *rgba8888_pixels, const int pixel_format, void *dst_pixels, const int pixel_num);

/**
 * テクスチャ用構造体
 */
typedef struct Texture {
    /**
     * 画像幅
     */
    int width;
    
    /**
     * 画像高さ
     */
    int height;
    
    /**
     * GL側のテクスチャID
     */
    GLuint id;
} Texture;

/**
 * 引数sizeがpotならtrueを返す。
 */
extern bool Texture_checkPowerOfTwo(const int size);

/**
 * 引数width/height共にがpotならtrueを返す。
 */
extern bool Texture_checkPowerOfTwoWH(const int width, const int height);

/**
 * 画像をテクスチャとして読み込む。
 * 読み込んだ画像はes20_freeTexture()で解放する
 */
extern Texture* Texture_load(GLApplication *app, const char* file_name, const int pixel_fotmat);

/**
 * テクスチャの縦横が2のn乗であればtrueを返す
 */
extern bool Texture_isPowerOfTwo(Texture *texture);

/**
 * テクスチャを解放する。
 */
extern void Texture_free(Texture *texture);


/**
 * 圧縮テクスチャ ETC1-PKM形式
 * Android標準サポート
 */
#define TEXTURE_COMPRESS_ETC1         10

/**
 * 圧縮テクスチャ PVRTC形式
 * PowerVR系GPUがサポート
 */
#define TEXTURE_COMPRESS_PVRTC        11

/**
 * 圧縮テクスチャ Khronos Texture形式
 * Tegra(DXT)/Adreno(ATC)がサポート
 */
#define TEXTURE_COMPRESS_KTX          12


/**
 * PKMフォーマット画像
 * Android標準ツールで作成可能
 */
typedef struct PkmImage {
    /**
     * origin data
     */
    RawData *raw;
    
    /**
     * 解像度が2のn乗であるかのチェック。
     * TEXTURE_POT | TEXTURE_NPOT
     */
    int power_of_two;
    
    /**
     * 画像幅
     */
    int width;
    
    /**
     * 画像高さ
     */
    int height;
    
    /**
     * 圧縮前の画像幅
     */
    int origin_width;
    
    /**
     * 圧縮前の画像高さ
     */
    int origin_height;
    
    /**
     * ETC1 = ETC1_RGB_NO_MIPMAPS = 0
     */
    int16_t data_type;
    
    /**
     * 圧縮テクスチャへのポインタ
     */
    void* image;
    
    /**
     * 圧縮テクスチャ本体の長さ
     */
    int image_bytes;
} PkmImage;


/**
 * PKM圧縮画像を読み込む。
 * 読み込んだ画像はPkmImage_free()で解放する
 */
extern PkmImage* PkmImage_load(GLApplication *app, const char* file_name);

/**
 * PKM圧縮画像を解放する
 */
extern void PkmImage_free(GLApplication *app, PkmImage *pkm);


/**
 * PVRTCフォーマット画像
 * iOS標準ツールで作成可能
 */
typedef struct PvrtcImage {
    /**
     * 読み込み元のデータ
     */
    RawData *raw;
    
    /**
     * PVRTCフォーマット。下記のどれかが設定される。
     * GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG
     * GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG
     */
    GLenum format;
    
    /**
     * 1pixelごとのビット数
     */
    int bits_per_pixel;
    
    /**
     * 解像度が2のn乗であるかのチェック。
     * PVRTCは仕様上、常にES20_POTとなる。
     */
    int power_of_two;
    
    /**
     * 画像幅
     * mipmapを含む場合、最大（miplevel 0）の値が格納される
     * PVRTCは仕様上、常にwidth == heightとなる。
     */
    int width;
    
    /**
     * 画像高さ
     * mipmapを含む場合、最大（miplevel 0）の値が格納される
     * PVRTCは仕様上、常にwidth == heightとなる。
     */
    int height;
    
    /**
     * 画像のmipmap数
     */
    int mipmaps;
    
    /**
     * 画像の長さ
     * mipmap数だけ格納されている
     */
    int* image_length_table;
    
    /**
     * 各画像へのポインタ
     * mipmap数だけ格納されている
     */
    void** image_table;
} PvrtcImage;

/**
 * PVRTC圧縮画像を読み込む
 * 読み込んだ画像はes20_freePvrtcImage()で解放する
 */
extern PvrtcImage* PvrtcImage_load(GLApplication *app, const char *file_name);

/**
 * PVRTC圧縮画像を解放する
 */
extern void PvrtcImage_free(GLApplication *app, PvrtcImage *pvrtc);


/**
 * KTXフォーマットデータ
 * 詳細：http://www.khronos.org/opengles/sdk/tools/KTX/file_format_spec/
 * Qualcomm Texture Tool(Windows専用）で作成可能
 */
typedef struct KtxImage {
    /**
     * origin data
     */
    RawData *raw;
    
    /**
     * 格納されたテクスチャのフォーマット
     * RGB, RGBA, BGRA, etc.
     */
    GLenum format;
    
    /**
     * 格納されたテクスチャのピクセルタイプ
     * UNSIGNED_BYTE, UNSIGNED_SHORT_5_6_5, etc.
     * 必ず0以外の値が格納される
     */
    GLenum type;
    
    /**
     * 画像幅
     * mipmapを含む場合、最大（miplevel 0）の値が格納される
     */
    int width;
    
    /**
     * 画像高さ
     * mipmapを含む場合、最大（miplevel 0）の値が格納される
     */
    int height;
    
    /**
     * 画像のmipmap数
     */
    int mipmaps;
    
    /**
     * 画像の長さ
     * mipmap数だけ格納されている
     */
    int* image_length_table;
    
    /**
     * 各画像へのポインタ
     * mipmap数だけ格納されている
     */
    void** image_table;
} KtxImage;


/**
 * Qualcomm Texture Toolを利用して作成したKTXファイルを読み込む
 */
extern KtxImage* KtxImage_load(GLApplication *app, const char* file_name);

/**
 * KTXファイルを解放する
 */
extern void KtxImage_free(GLApplication *app, KtxImage *image);

/**
 * PMDファイルのヘッダ情報
 */
typedef struct PmdHeader {
    /**
     * バージョン番号
     */
    GLfloat version;
    
    /**
     * モデル名
     */
    GLchar name[32];
    
    /**
     * コメント
     */
    GLchar comment[256 + 128];
} PmdHeader;

/**
 * PMDファイルが格納する頂点情報
 */
typedef struct PmdVertex {
    /**
     * 位置
     */
    vec3 position;
    
    /**
     * テクスチャUV
     */
    vec2 uv;
    
    /**
     * 法線
     */
    vec3 normal;
    
    /**
     * サンプルでは使用しないが、PMDファイル仕様的に含まれている情報
     */
    struct {
        
        /**
         * 関連付けられたボーン番号
         */
        GLshort bone_num[2];
        
        /**
         * ボーンの重み情報（最大100）
         * bone_num[0]の影響度がbone_weight、bone_num[1]の影響度が100 - bone_weightで表される
         */
        GLbyte bone_weight;
        
        /**
         * エッジ表示フラグ
         * 0=無効
         * 1=有効
         */
        GLbyte edge_flag;
    } extra;
} PmdVertex;

/**
 * PMD材質情報
 */
typedef struct PmdMaterial {
    /**
     * 拡散反射光
     */
    vec4 diffuse;
    
    /**
     * 頂点数
     */
    GLint indices_num;
    
    /**
     * テクスチャファイル名
     */
    GLchar diffuse_texture_name[20 + 12];
    
    /**
     * サンプルでは使用しないが、PMDファイル仕様的に含まれている情報
     */
    struct {
        
        /**
         * 輪郭フラグ
         */
        GLbyte edge_flag;
        
        /**
         *
         */
        GLfloat shininess;
        
        /**
         * スペキュラ色
         */
        vec3 specular_color;
        
        /**
         * 環境光
         */
        vec3 ambient_color;
        /**
         *
         */
        GLbyte toon_index;
        
        /**
         * エフェクト用テクスチャ名
         * .sph/.spaファイルが指定される。サンプル内では利用しない
         */
        GLchar effect_texture_name[20 + 12];
    } extra;
    
} PmdMaterial;

/**
 * ボーン情報
 */
typedef struct PmdBone {
    /**
     * ボーン名
     */
    GLchar name[20 + 12];
    
    /**
     * 親ボーン番号
     * 無い場合は0xFFFF = -1
     */
    GLshort parent_bone_index;
    
    /**
     * ボーンの位置
     */
    vec3 position;
    
    /**
     * サンプルでは使用しないが、PMDファイル仕様的に含まれている情報
     */
    struct {
        /**
         * 制御ボーン
         */
        GLshort tail_pos_bone_index;
        
        /**
         * ボーンの種類
         */
        GLbyte type;
        
        /**
         * IKボーン
         * サンプルではベイク済みもしくは手動制御を前提とする
         */
        GLshort ik_parent_bone_index;
    } extra;
} PmdBone;

/**
 * PMDファイルコンテナ
 */
typedef struct PmdFile {
    /**
     * ヘッダファイル
     */
    PmdHeader header;
    
    /**
     * 頂点配列
     */
    PmdVertex *vertices;
    
    /**
     * 頂点数
     */
    GLuint vertices_num;
    
    /**
     * 頂点インデックス
     */
    GLushort *indices;
    
    /**
     * 頂点インデックス数
     */
    GLuint indices_num;
    
    /**
     * 材質情報
     */
    PmdMaterial *materials;
    
    /**
     * 管理している材質数
     */
    GLuint materials_num;
    
    /**
     * ボーン情報
     */
    PmdBone *bones;
    
    /**
     * ボーン数
     */
    GLuint bones_num;
} PmdFile;

/**
 * テクスチャ名とTexture構造体のマッピングを行う
 */
typedef struct PmdTextureList {
    /**
     * テクスチャ名配列
     */
    GLchar **texture_names;
    
    /**
     * テクスチャの実体配列
     */
    Texture **textures;
    
    /**
     * 管理しているテクスチャ数
     */
    int textures_num;
} PmdTextureList;

/**
 * PMDファイルを生成する
 */
extern PmdFile* PmdFile_create(RawData *data);

/**
 * PMDファイルをロードする
 */
extern PmdFile* PmdFile_load(GLApplication *app, const char* file_name);

/**
 * PMDファイルを解放する
 */
extern void PmdFile_free(PmdFile *pmd);

/**
 * 最小最大地点を求める
 */
extern void PmdFile_calcAABB(PmdFile *pmd, vec3 *minPoint, vec3 *maxPoint);

/**
 * PMDファイル内のテクスチャを列挙する
 *
 * 注意）
 * PMDの性質上、テクスチャファイルはtgaやbmp等の巨大ファイルになる恐れがある。
 * そのため、pngに変換したファイルをhoge.bmp.pngのような形で".png"を付けて配置する。
 * 処理をラクにするため、PNGファイルの場合もhoge.png.pngのように共通化する。
 */
extern PmdTextureList* PmdFile_createTextureList(GLApplication *app, PmdFile *pmd);

/**
 * 指定した名前のテクスチャを取得する
 */
extern Texture* PmdFile_getTexture(PmdTextureList *texList, const GLchar *name);

/**
 * 管理しているテクスチャを解放する
 */
extern void PmdFile_freeTextureList(PmdTextureList* texList);

#endif /* data_h */
