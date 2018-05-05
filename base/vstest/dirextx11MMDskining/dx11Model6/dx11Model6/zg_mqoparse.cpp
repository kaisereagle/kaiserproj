#include "stdafx.h"

#include "zg_mqo.h"


//iso8859_1.hpp : warning C4819: �t�@�C���́A���݂̃R�[�h �y�[�W (932) �ŕ\���ł��Ȃ��������܂�ł��܂��B�f�[�^�̑�����h�����߂ɁA�t�@�C���� Unicode �`���ŕۑ����Ă��������B
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
//�K�v�Ȃ��̂����C���X�^���X��
qi::char_type char_;
qi::int_type int_;
qi::float_type float_;
qi::space_type space;
qi::blank_type blank;
qi::unused_type unused;
#endif
//BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
//20%���炢�R���p�C���������Ȃ�

//--------------------------------------
struct ParseContext
{
	ParseContext(MQOModel& mqo) : MQO(mqo), curObject(0){}

	MQOModel& MQO;
	zgU32 curObject;	// ���ݍ쐬����Object�C���f�b�N�X
private:
	ParseContext();
};


//-----------------------------------------------------
// �s���@���s��\r\n�i���^�Z�R�C�A�̃f�[�^�d�l�j
template<typename Iterator, typename A1=qi::unused_type>
struct endline : qi::grammar<Iterator, A1()>
{
    endline() : endline::base_type(r)
    {
		//��0��ȏ�ˉ��s�R�[�h
        r = *blank >> char_("\r\n");
    }
    qi::rule<Iterator, A1()> r;

	//A1=qi::unused_type�̏ꍇ�����Ȃ�
};

//-----------------------------------------------------
//"������" ������""�ň͂܂ꂽ����
template<typename Iterator, typename A1=qi::unused_type>
struct dqstring : qi::grammar<Iterator, A1()>
{
    dqstring() : dqstring::base_type(r)
    {
        r = '\"' >> *(char_ - char_('\"')) >> '\"';
    }
	//'\"'�̍��́A�������Ȃ����ߑ���A1�ɂ͊܂܂�Ȃ��i���Ԃ�j
	//���܂藝�������Ɏg�p�A���܂܂��]�ޓ���ɂȂ��Ă��邾������
	qi::rule<Iterator, A1()> r;
};

template<typename Iterator>
bool skip_chunk(Iterator& first, Iterator last);
template<typename Iterator>
bool skip_mline_chunk_end(Iterator& first, Iterator last);

//-----------------------------------------------------
// �`�����N���X�L�b�v
template<typename Iterator>
bool skip_chunk(Iterator& first, Iterator last)
{
	endline<Iterator> eline;
	dqstring<Iterator> dqstr;
	char term = 0;
	
	// �`�����N�̍s����"{"�����s�݂̂�
	if( !qi::parse(first,last,
					*space			// ��(�X�y�[�X�A�^�u�A���s�Ȃǁj�X�L�b�v 
					>> *( +(char_ - char_("{}\"") - eline) | dqstr ) //���䕶���������C�ӕ�����or"������(���䕶���܂�)"
					>> -char_('{')	// {��0or1��@���ʂ�term�ɏo��
					>> eline		// ���s(\r\n)
					, qi::unused, qi::unused, term) )
	{
		return false;//�����G���[
	}
	if(term == '{'){//{�ŏI��
		// }�܂ŃX�L�b�v
		if( !skip_mline_chunk_end(first, last) ){
			//{}�����ĂȂ�
			return false;
		}
	}
	return true;
}

//-----------------------------------------------------
// "{"��͌�̕����s�`�����N�I���܂ŃX�L�b�v 
template<typename Iterator>
bool skip_mline_chunk_end(Iterator& first, Iterator last)
{
	// �l�X�g���ꂽ�`�����N�̃X�L�b�v
	while( skip_chunk(first, last) ){
	}
	// �I���m�F
	if( !qi::parse( first, last, *space >> char_('}')) ){
		//{}�����ĂȂ�
		return false;
	}
	return true;
}

//-----------------------------------------------------
//�}�e���A���p�����[�^��� float[n]
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
		// ���l�̌��`�F�b�N�Ȃ�
		// fv_num������Ȃ��ꍇ�A��͓r���ŏI��
	}
	return true;
}

//-----------------------------------------------------
//�}�e���A���p�����[�^��� string
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
#if 0//����
	if( !qi::parse(first, last, dqstr >> *blank >> char_(')') , str) ){
	�Ƃ���ƕ�����̌���")"���ǉ������
	�������P�����w�聕rule�ɑΉ����鑮��������Ȃ��ꍇ
	�������ǉ�����Ă���
	���̏ꍇ char_(')')�ɑΉ����鑮�����Ȃ�����str��")"���ǉ�����Ă��܂�
#endif
	return true;
}

//-----------------------------------------------------
//Material�`�����N���
template <typename Iterator>
bool parse_material(Iterator& first, Iterator last, ParseContext& ctx)
{
	endline<Iterator> eline;
	dqstring<Iterator, std::string> dqstr;

	// Material mat_num {\r\n
	int mat_num = 0;
	if( !qi::parse(first, last,
				"Material" >> +blank >>int_ >> *blank >> "{" >> eline,	//rule
							//�@"Material"�̂悤�Ȓl�i���e�����j�ɂ͑������Ȃ��̂ő����������Ȃ�
				unused,		// +blank�̑���
				mat_num,	// qi::int_�̑���
				unused)		// *blank�̑��� �Ȃ��Ă�����
	){
		return false;
	}

	ctx.MQO.aMaterial.reserve(mat_num);

	for(int m=0;m<mat_num;++m){
		MQOMaterial mat;

		// "�}�e���A����"
		std::string name;// dqstr�̑����A"�����������g���󂯎��
		if( !qi::parse(first, last, *space >> dqstr >> +blank, unused, name) ){
			return false;
		}
		mat.strName = name;

		float fv[4];
		std::string tex_name;

		// �p�����[�^�̏��Ԃ͌Œ�Ƃ���
		//�@�����łȂ��ꍇqi::symbols���g���ď����̕���

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

		//���̍s�܂ňړ�
		if( !qi::parse(first, last, *(char_ - eline) >> eline) ){
			return false;
		}
		
		// �}�e���A���ǉ�
		ctx.MQO.aMaterial.push_back( mat );
	}

	//Material�`�����N�I��
	return skip_mline_chunk_end(first, last);
}

//-----------------------------------------------------
// vertex�`�����N���
template <typename Iterator>
bool parse_vertex(Iterator& first, Iterator last, ParseContext& ctx)
{
	// �쐬���̃I�u�W�F�N�g
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
		// ���_�ǉ�
		obj.aVertex.push_back(ZGVECTOR3(fv[0], fv[1], fv[2]));
	}
	return qi::parse(first, last, *space >> "}");
}

//-----------------------------------------------------
// face�`�����N���
template <typename Iterator>
bool parse_face(Iterator& first, Iterator last, ParseContext& ctx)
{
	// �쐬���̃I�u�W�F�N�g
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
		// �t�F�C�X�i�|���S���j�ǉ�

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

		// �t�F�C�X�ǉ�
		for(zgU32 p=0;p<(zgU32)(poly-2);++p){
			//�l�p�`�ȏ�͎O�p�`�ɕϊ�
			zgU32 t2 = 0;			//���W�n���قȂ�̂Ŕ��Ε�����		
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
// Object�`�����N���
template <typename Iterator>
bool parse_object(Iterator& first, Iterator last, ParseContext& ctx)
{
	endline<Iterator> eline;
	dqstring<Iterator, std::string> dqstr;

	// Object �hobjname" {\r\n
	std::string obj_name;
	if( !qi::parse(first, last,
				"Object" >> +blank >> dqstr >> *blank >> "{" >> eline,	//rule
				unused,		// +blank�̑���
				obj_name)	// dqstr�̑���
	){
		return false;
	}

	// �I�u�W�F�N�g�ǉ�
	ctx.MQO.aObject.push_back( MQOObject() );
	ctx.curObject = ctx.MQO.aObject.size() - 1;
	MQOObject& obj = ctx.MQO.aObject[ctx.curObject]; 
	obj.strName = obj_name;

	//�V���{��
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
	//���̃`�����N�͖���
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
			//���Ή��`�����N
			if( !skip_chunk(first, last) ){
				return false;//�G���[
			}
		}
	}

	//Object�`�����N�I��
	return true;
}

//--------------------------------------------------
template <typename Iterator>
bool parse_mqo(Iterator& first, Iterator last, ParseContext& ctx)
{
	//�w�b�_�`�F�b�N
	if( !qi::parse(first, last,
				"Metasequoia Document\r\nFormat Text Ver 1.0\r\n")
	){
		return false;
	}

	//�V���{��
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
			break;//�I���
		}

		//�󔒃X�L�b�v
		std::string attr_str;//���������p
		if( !qi::parse(first, last, *space, attr_str) ){
			return false;//�G���[
		}
		// attr_str�ɋ󔒁i�X�y�[�X�A�^�u�A���s�Ȃǁj�����񂪒ǉ������@�ˑ���
		// �����ł͎g��Ȃ��A�����p
		// �������ȗ�����Ƒ������󂯎��Ȃ�

		int chunk;
		const char* chunk_ptr = first;//
		// sym_main�ɒǉ������`�����N���Ƃ̈�v
		if( qi::parse(chunk_ptr, last, sym_main >> +space, chunk) ){
			//chunk��sym_main�̑������󂯎��@("Scene", CH_SCENE������)

			// �`�����N���
			// parse��chunk_ptr��������������@ptr�ł�����x�`�����N�擪������
			bool result = false;
			switch(chunk){
			case CH_SCENE:// Scene ��͂��Ȃ�
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
				return false;//�G���[ 
			}
		}else{
			// ���Ή��`�����N
			if( !skip_chunk(first, last) ){
				return false;//�G���[
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

