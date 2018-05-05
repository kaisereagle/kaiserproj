#include "stdafx.h"

#include "zg_mqo.h"


//iso8859_1.hpp : warning C4819: ファイルは、現在のコード ページ (932) で表示できない文字を含んでいます。データの損失を防ぐために、ファイルを Unicode 形式で保存してください。
#pragma warning(disable : 4819) 
#define BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
#include <boost/spirit/include/qi.hpp>

namespace qi = boost::spirit::qi;

#ifndef BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
using qi::char_;
using qi::int_;
using qi::float_;
using qi::space;
using qi::blank;
using qi::unused;
#endif

namespace{

#ifdef BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
//必要なものだけインスタンス化
qi::char_type char_;
qi::int_type int_;
qi::float_type float_;
qi::space_type space;
qi::blank_type blank;
qi::unused_type unused;
#endif
//BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
//20%ぐらいコンパイルが速くなる

//--------------------------------------
struct ParseContext
{
	ParseContext(MQOModel& mqo) : MQO(mqo), curObject(0){}

	MQOModel& MQO;
	zgU32 curObject;	// 現在作成中のObjectインデックス
private:
	ParseContext();
};


//-----------------------------------------------------
// 行末　改行は\r\n（メタセコイアのデータ仕様）
template<typename Iterator, typename A1=qi::unused_type>
struct endline : qi::grammar<Iterator, A1()>
{
    endline() : endline::base_type(r)
    {
		//空白0回以上⇒改行コード
        r = *blank >> char_("\r\n");
    }
    qi::rule<Iterator, A1()> r;

	//A1=qi::unused_typeの場合属性なし
};

//-----------------------------------------------------
//"文字列" 属性は""で囲まれた部分
template<typename Iterator, typename A1=qi::unused_type>
struct dqstring : qi::grammar<Iterator, A1()>
{
    dqstring() : dqstring::base_type(r)
    {
        r = '\"' >> *(char_ - char_('\"')) >> '\"';
    }
	//'\"'の項は、属性がないため属性A1には含まれない（たぶん）
	//あまり理解せずに使用、たままた望む動作になっているだけかも
	qi::rule<Iterator, A1()> r;
};

template<typename Iterator>
bool skip_chunk(Iterator& first, Iterator last);
template<typename Iterator>
bool skip_mline_chunk_end(Iterator& first, Iterator last);

//-----------------------------------------------------
// チャンクをスキップ
template<typename Iterator>
bool skip_chunk(Iterator& first, Iterator last)
{
	endline<Iterator> eline;
	dqstring<Iterator> dqstr;
	char term = 0;
	
	// チャンクの行末が"{"か改行のみか
	if( !qi::parse(first,last,
					*space			// 空白(スペース、タブ、改行など）スキップ 
					>> *( +(char_ - char_("{}\"") - eline) | dqstr ) //制御文字を除く任意文字列or"文字列(制御文字含む)"
					>> -char_('{')	// {が0or1回　結果をtermに出力
					>> eline		// 改行(\r\n)
					, qi::unused, qi::unused, term) )
	{
		return false;//書式エラー
	}
	if(term == '{'){//{で終了
		// }までスキップ
		if( !skip_mline_chunk_end(first, last) ){
			//{}が閉じてない
			return false;
		}
	}
	return true;
}

//-----------------------------------------------------
// "{"解析後の複数行チャンク終了までスキップ 
template<typename Iterator>
bool skip_mline_chunk_end(Iterator& first, Iterator last)
{
	// ネストされたチャンクのスキップ
	while( skip_chunk(first, last) ){
	}
	// 終了確認
	if( !qi::parse( first, last, *space >> char_('}')) ){
		//{}が閉じてない
		return false;
	}
	return true;
}

//-----------------------------------------------------
//マテリアルパラメータ解析 float[n]
template <typename Iterator>
bool parse_material_paramfv(Iterator& first, Iterator last, const char* const param_name, float* fv, int fv_num )
{
	for(int i=0;i<fv_num;++i){ fv[i] = 0.0f;}

	// param_name(
	if( !qi::parse(first, last, *blank >> param_name >> *blank >> "(" >> *blank) ){
		return false;
	}
	// float ... float)
	for(int i=0;i<fv_num;++i){
		if( !qi::parse(first, last, float_ >> *blank >> -char_(')') , fv[i]) ){
			return false;
		}
		// 数値の個数チェックなし
		// fv_numが足りない場合、解析途中で終了
	}
	return true;
}

//-----------------------------------------------------
//マテリアルパラメータ解析 string
template <typename Iterator>
bool parse_material_paramstr(Iterator& first, Iterator last, const char* const param_name, std::string& str )
{
	dqstring<Iterator, std::string> dqstr;
	str = "";
	// param_name(
	if( !qi::parse(first, last, *blank >> param_name >> *blank >> "(" >> *blank) ){
		return false;
	}
	// "str")
	if( !qi::parse(first, last, dqstr >> *blank >> ')' , str, unused) ){//dqstr>str *blank>unused
		return false;
	}
#if 0//属性
	if( !qi::parse(first, last, dqstr >> *blank >> char_(')') , str) ){
	とすると文字列の後ろに")"が追加される
	属性を１つだけ指定＆ruleに対応する属性が足りない場合
	属性が追加されていく
	この場合 char_(')')に対応する属性がないためstrに")"が追加されてしまう
#endif
	return true;
}

//-----------------------------------------------------
//Materialチャンク解析
template <typename Iterator>
bool parse_material(Iterator& first, Iterator last, ParseContext& ctx)
{
	endline<Iterator> eline;
	dqstring<Iterator, std::string> dqstr;

	// Material mat_num {\r\n
	int mat_num = 0;
	if( !qi::parse(first, last,
				"Material" >> +blank >>int_ >> *blank >> "{" >> eline,	//rule
							//　"Material"のような値（リテラル）には属性がないので属性を書かない
				unused,		// +blankの属性
				mat_num,	// qi::int_の属性
				unused)		// *blankの属性 なくてもいい
	){
		return false;
	}

	ctx.MQO.aMaterial.reserve(mat_num);

	for(int m=0;m<mat_num;++m){
		MQOMaterial mat;

		// "マテリアル名"
		std::string name;// dqstrの属性、"を除いた中身を受け取る
		if( !qi::parse(first, last, *space >> dqstr >> +blank, unused, name) ){
			return false;
		}
		mat.strName = name;

		float fv[4];
		std::string tex_name;

		// パラメータの順番は固定とする
		//　そうでない場合qi::symbolsを使って処理の分岐

		// col(float float float float)
		if( !parse_material_paramfv(first, last, "col", fv, 4) ){
			return false;
		}else{
			mat.v4Col = ZGVECTOR4( fv[0],fv[1],fv[2],fv[3] );
		}
		// dif(float)
		if( !parse_material_paramfv(first, last, "dif", fv, 1) ){ return false;}

		// amb(float)
		if( !parse_material_paramfv(first, last, "amb", fv, 1) ){ return false;}

		// emi(float)
		if( !parse_material_paramfv(first, last, "emi", fv, 1) ){ return false;}

		// spc(float)
		if( !parse_material_paramfv(first, last, "spc", fv, 1) ){ return false;}

		// power(float)
		if( !parse_material_paramfv(first, last, "power", fv, 1) ){ return false;}

		// tex("texture")
		if( parse_material_paramstr(first, last, "tex", tex_name) ){
			mat.strTexName = tex_name;
		}

		//次の行まで移動
		if( !qi::parse(first, last, *(char_ - eline) >> eline) ){
			return false;
		}
		
		// マテリアル追加
		ctx.MQO.aMaterial.push_back( mat );
	}

	//Materialチャンク終了
	return skip_mline_chunk_end(first, last);
}

//-----------------------------------------------------
// vertexチャンク解析
template <typename Iterator>
bool parse_vertex(Iterator& first, Iterator last, ParseContext& ctx)
{
	// 作成中のオブジェクト
	if(ctx.curObject >= ctx.MQO.aObject.size()){ return false;}
	MQOObject& obj = ctx.MQO.aObject[ctx.curObject];

	endline<Iterator> eline;
	// vertex vertex_num {\r\n
	int v_num = 0;
	if( !qi::parse(first, last,
				"vertex" >> +blank >> int_ >> *blank >> "{" >> eline,	//rule
				unused,	v_num )
	){
		return false;
	}
	obj.aVertex.reserve(v_num);

	// float float float\r\n
	for(int v=0;v<v_num;++v){
		float fv[3];
		if( !qi::parse(first, last,
					*space >> float_ >> +blank >> float_ >> +blank >> float_ >> eline,
					unused, fv[0], unused, fv[1], unused, fv[2])
		){
			return false;
		}
		// 頂点追加
		obj.aVertex.push_back(ZGVECTOR3(fv[0], fv[1], fv[2]));
	}
	return qi::parse(first, last, *space >> "}");
}

//-----------------------------------------------------
// faceチャンク解析
template <typename Iterator>
bool parse_face(Iterator& first, Iterator last, ParseContext& ctx)
{
	// 作成中のオブジェクト
	if(ctx.curObject >= ctx.MQO.aObject.size()){ return false;}
	MQOObject& obj = ctx.MQO.aObject[ctx.curObject];

	endline<Iterator> eline;
	// face face_num {\r\n
	int f_num = 0;
	if( !qi::parse(first, last,
				"face" >> +blank >> int_ >> *blank >> "{" >> eline,	//rule
				unused,	f_num )
	){
		return false;
	}
	obj.aFace.reserve(f_num);

	// int V(int int ... int) M(int) UV(float float .... float float)
	for(int f=0;f<f_num;++f){
		// フェイス（ポリゴン）追加

		// int V(
		zgU32 poly = 0;
		if( !qi::parse(first, last, *space >> int_ >> +blank >> "V" >> *blank >> "(" >> *blank, unused, poly) ){
			return false;
		}

		// int int ... int
		std::vector<zgU32> vids;
		vids.reserve(poly);
		zgU32 vidx = 0;
		for(zgU32 p=0;p<poly;++p){
			if( !qi::parse(first, last, *blank >> int_, unused, vidx) ){
				return false;
			}
			vids.push_back(vidx);
		}
		// )
		if( !qi::parse(first, last, *blank >> ")") ){ return false; }

		// M(
		int midx = 0;
		if( qi::parse(first, last, *blank >> "M" >> *blank >> "(" >> *blank) ){
			// int)
			if( !qi::parse(first, last, int_ >> *blank >> ")", midx) ){ return false;}
		}

		// UV(
		std::vector<ZGVECTOR2> uvs;
		uvs.reserve(poly);
		if( qi::parse(first, last, *blank >> "UV" >> *blank >> "(" >> *blank) ){
			// float .... float
			float uv[2]={0,0};
			for(zgU32 p=0;p<poly;++p){
				if( !qi::parse(first, last, *blank >> float_ >> +blank >> float_, unused, uv[0], unused, uv[1]) ){
					return false;
				}
				uvs.push_back(ZGVECTOR2(uv[0], uv[1]));
			}
			// )
			if( !qi::parse(first, last, *blank >> ")") ){ return false; }
		}

		// フェイス追加
		for(zgU32 p=0;p<(zgU32)(poly-2);++p){
			//四角形以上は三角形に変換
			zgU32 t2 = 0;			//座標系が異なるので反対方向に		
			zgU32 t1 = (p+1)%poly;
			zgU32 t0 = (p+2)%poly;
			zgU32 i0 = vids[t0];
			zgU32 i1 = vids[t1];
			zgU32 i2 = vids[t2];

			obj.aFace.push_back(MQOFace());
			MQOFace& face = obj.aFace.back();
			face.idxMat = midx;
			face.aVidx[0] = i0;
			face.aVidx[1] = i1;
			face.aVidx[2] = i2;
			if(uvs.size()>=poly){
				face.aUV[0] = uvs[t0];
				face.aUV[1] = uvs[t1];
				face.aUV[2] = uvs[t2];
			}else{
				face.aUV[0] = ZGVECTOR2(0,0);
				face.aUV[1] = ZGVECTOR2(0,0);
				face.aUV[2] = ZGVECTOR2(0,0);
			}
			face.aNor[0] = ZGVECTOR3(0,1,0);
			face.aNor[1] = ZGVECTOR3(0,1,0);
			face.aNor[2] = ZGVECTOR3(0,1,0);
			
		}

	} 

	return qi::parse(first, last, *space >> "}");
}

//-----------------------------------------------------
// Objectチャンク解析
template <typename Iterator>
bool parse_object(Iterator& first, Iterator last, ParseContext& ctx)
{
	endline<Iterator> eline;
	dqstring<Iterator, std::string> dqstr;

	// Object ”objname" {\r\n
	std::string obj_name;
	if( !qi::parse(first, last,
				"Object" >> +blank >> dqstr >> *blank >> "{" >> eline,	//rule
				unused,		// +blankの属性
				obj_name)	// dqstrの属性
	){
		return false;
	}

	// オブジェクト追加
	ctx.MQO.aObject.push_back( MQOObject() );
	ctx.curObject = ctx.MQO.aObject.size() - 1;
	MQOObject& obj = ctx.MQO.aObject[ctx.curObject]; 
	obj.strName = obj_name;

	//シンボル
	enum {
		OCH_VERTEX = 1,
		OCH_FACE,
		OCH_FACET,
	};
	qi::symbols<char, int> sym_obj;
	sym_obj.add
		("vertex", OCH_VERTEX)
		("face", OCH_FACE)
		("facet", OCH_FACET);
	//他のチャンクは無視
	while(first!=last){
		if( qi::parse(first, last, *space >> "}") ){
			break;
		}
		if( ! qi::parse(first, last, *space) ){ return false;}
		int chunk = 0;
		Iterator chunk_ptr = first;//
		if( qi::parse(chunk_ptr, last, sym_obj >> +space, chunk, unused) ){
			switch(chunk){
			case OCH_VERTEX:
				if( !parse_vertex(first, last, ctx) ){ return false;}
				break;
			case OCH_FACE:
				if( !parse_face(first, last, ctx) ){ return false;}
				break;
			case OCH_FACET:
				{// facet float
					float facet = 0.0f;
					if( ! qi::parse(chunk_ptr, last, *space>>float_,unused, facet) ){ return false;}
					obj.radFacet = facet*3.141592654f/180.0f;
					if( !skip_chunk(first, last) ){ return false;}
				}
				break;
			default:
				if( !skip_chunk(first, last) ){ return false;}
				break;
			}
		}else{
			//未対応チャンク
			if( !skip_chunk(first, last) ){
				return false;//エラー
			}
		}
	}

	//Objectチャンク終了
	return true;
}

//--------------------------------------------------
template <typename Iterator>
bool parse_mqo(Iterator& first, Iterator last, ParseContext& ctx)
{
	//ヘッダチェック
	if( !qi::parse(first, last,
				"Metasequoia Document\r\nFormat Text Ver 1.0\r\n")
	){
		return false;
	}

	//シンボル
	enum {
		CH_SCENE = 1,
		CH_MATERIAL,
		CH_OBJECT,
	};
	qi::symbols<char, int> sym_main;
	sym_main.add
		("Scene", CH_SCENE)
		("Material", CH_MATERIAL)
		("Object", CH_OBJECT);

	while(1){
		if( qi::parse(first, last, *space >> "Eof") ){
			break;//終わり
		}

		//空白スキップ
		std::string attr_str;//属性実験用
		if( !qi::parse(first, last, *space, attr_str) ){
			return false;//エラー
		}
		// attr_strに空白（スペース、タブ、改行など）文字列が追加される　⇒属性
		// ここでは使わない、実験用
		// 引数を省略すると属性を受け取らない

		int chunk;
		const char* chunk_ptr = first;//
		// sym_mainに追加したチャンク名との一致
		if( qi::parse(chunk_ptr, last, sym_main >> +space, chunk) ){
			//chunkでsym_mainの属性を受け取る　("Scene", CH_SCENE←これ)

			// チャンク解析
			// parseでchunk_ptrが書き換えられる　ptrでもう一度チャンク先頭から解析
			bool result = false;
			switch(chunk){
			case CH_SCENE:// Scene 解析しない
				result = skip_chunk(first, last);
				break;
			case CH_MATERIAL:// Material
				result = parse_material(first, last, ctx);
				break;
			case CH_OBJECT:// Object
				result = parse_object(first, last, ctx);
				break;
			}
			if(!result){
				return false;//エラー 
			}
		}else{
			// 未対応チャンク
			if( !skip_chunk(first, last) ){
				return false;//エラー
			}
		}
	}

	ComputeNormal(ctx.MQO);

	return true;
}

}//ns

//--------------------------------------------
bool CreateMQO(const char* data, size_t size, MQOModel& mqo)
{
	const char* ptr = data;
	const char* const end = data + size;
	ParseContext ctx(mqo);
	return parse_mqo(ptr,end, ctx);
}

