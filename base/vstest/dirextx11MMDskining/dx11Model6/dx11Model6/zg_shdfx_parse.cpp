#include "stdafx.h"

#include "zg_shdfx_desc.h"

//iso8859_1.hpp : warning C4819: �t�@�C���́A���݂̃R�[�h �y�[�W (932) �ŕ\���ł��Ȃ��������܂�ł��܂��B�f�[�^�̑�����h�����߂ɁA�t�@�C���� Unicode �`���ŕۑ����Ă��������B
#pragma warning(disable : 4819) 
#define BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
#include <boost/spirit/include/qi.hpp>

namespace qi = boost::spirit::qi;

namespace{

#ifdef BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
//�K�v�Ȃ��̂����C���X�^���X��
qi::char_type char_;
qi::alnum_type alnum;
qi::alpha_type alpha;
qi::int_type int_;
qi::space_type space;
qi::blank_type blank;
qi::unused_type unused;
qi::eol_type eol;

#endif

//-----------------------------------------------------
//���O������
template<typename Iterator, typename A1=qi::unused_type>
struct namestr_type : qi::grammar<Iterator, A1()>
{
    namestr_type() : namestr_type::base_type(r)
    {
        r = (alpha|char_('_')) > *(alnum|char_('_')); 
    }
	qi::rule<Iterator, A1()> r;
};

// �R�����g
template<typename Iterator, typename A1=qi::unused_type>
struct comment_type : qi::grammar<Iterator, A1()>
{
   comment_type() : comment_type::base_type(r)
   {
		r = ( "//" >> *(char_ - eol) >> eol)
			|| ( "/*" >> *((*(char_-'*')>>+char_('*')) - '/') >> '/' )
			;
   }
   qi::rule<Iterator, A1()> r;
};

//�X�L�b�v�����i�󔒁A���s�A�R�����g�j
template<typename Iterator, typename A1=qi::unused_type>
struct skip_type : qi::grammar<Iterator, A1()>
{
    skip_type() : skip_type::base_type(r)
    {
		r = +space
			|| comment
			;
    }
	qi::rule<Iterator, A1()> r;
	comment_type<Iterator,A1> comment;
};



//�V���{��
enum PASS_ELEM{
	PASS_VERTEXSHADER = 1,
	PASS_PIXELSHADER,
	PASS_CULLMODE,
	
	PASS_BLEND_ENABLE,
	PASS_SRC_BLEND,
	PASS_DEST_BLEND,
	PASS_BLEND_OP,
	PASS_SRC_BLEND_ALPHA,
	PASS_DEST_BLEND_ALPHA,
	PASS_BLEND_OP_ALPHA,
	PASS_RT_WRITE_MASK,

	PASS_DEPTH_ENABLE,
	PASS_DEPTH_FUNC,
};

BOOL conv_bool(const std::string& val)
{
	return val=="TRUE";
}
D3D11_BLEND conv_blend(const std::string& val)
{
	//������if��
	if(val == "ZERO")					return D3D11_BLEND_ZERO;
	else if(val == "ONE")				return D3D11_BLEND_ONE;
	else if(val == "SRC_COLOR")			return D3D11_BLEND_SRC_COLOR;
	else if(val == "INV_SRC_COLOR")		return D3D11_BLEND_INV_SRC_COLOR;
	else if(val == "SRC_ALPHA")			return D3D11_BLEND_SRC_ALPHA;
	else if(val == "INV_SRC_ALPHA")		return D3D11_BLEND_INV_SRC_ALPHA;
	else if(val == "DEST_ALPHA")		return D3D11_BLEND_DEST_ALPHA;
	else if(val == "INV_DEST_ALPHA")	return D3D11_BLEND_INV_DEST_ALPHA;
	else if(val == "DEST_COLOR")		return D3D11_BLEND_DEST_COLOR;
	else if(val == "INV_DEST_COLOR")	return D3D11_BLEND_INV_DEST_COLOR;
	else if(val == "SRC_ALPHA_SAT")		return D3D11_BLEND_SRC_ALPHA_SAT;
	else if(val == "BLEND_FACTOR")		return D3D11_BLEND_BLEND_FACTOR;
	else if(val == "INV_BLEND_FACTOR")	return D3D11_BLEND_INV_BLEND_FACTOR;
	else if(val == "SRC1_COLOR")		return D3D11_BLEND_SRC1_COLOR;
	else if(val == "INV_SRC1_COLOR")	return D3D11_BLEND_INV_SRC1_COLOR;
	else if(val == "SRC1_ALPHA")		return D3D11_BLEND_SRC1_ALPHA;
	else if(val == "INV_SRC1_ALPHA")	return D3D11_BLEND_INV_SRC1_ALPHA;

	return D3D11_BLEND_ZERO;
}
D3D11_BLEND_OP conv_blendop(const std::string& val)
{
	if(val == "ADD")				return D3D11_BLEND_OP_ADD;
	else if(val == "SUBTRACT")		return D3D11_BLEND_OP_SUBTRACT;
	else if(val == "REV_SUBTRACT")	return D3D11_BLEND_OP_REV_SUBTRACT;
	else if(val == "MIN")			return D3D11_BLEND_OP_MIN;
	else if(val == "MAX")			return D3D11_BLEND_OP_MAX;
	return D3D11_BLEND_OP_ADD;
}

zgU32 conv_write_mask(const std::string& val)
{
	if(val == "RED")		return D3D11_COLOR_WRITE_ENABLE_RED;
	else if(val == "GREEN")	return D3D11_COLOR_WRITE_ENABLE_GREEN;
	else if(val == "BLUE")	return D3D11_COLOR_WRITE_ENABLE_BLUE;
	else if(val == "ALPHA")	return D3D11_COLOR_WRITE_ENABLE_ALPHA;
	else if(val == "ALL")	return D3D11_COLOR_WRITE_ENABLE_ALL;
	
	// RGBA�`��
	zgU32 ret = 0;
	for(char c:val){
		if(c == 'r' || c == 'R')ret |= D3D11_COLOR_WRITE_ENABLE_RED;
		if(c == 'g' || c == 'G')ret |= D3D11_COLOR_WRITE_ENABLE_GREEN;
		if(c == 'b' || c == 'B')ret |= D3D11_COLOR_WRITE_ENABLE_BLUE;
		if(c == 'a' || c == 'A')ret |= D3D11_COLOR_WRITE_ENABLE_ALPHA;
	}
	return ret;
}

D3D11_COMPARISON_FUNC conv_func(const std::string& val)
{
	if(val == "NEVER")		return D3D11_COMPARISON_NEVER;
	else if(val == "LESS")	return D3D11_COMPARISON_LESS;
	else if(val == "EQUAL")	return D3D11_COMPARISON_EQUAL;
	else if(val == "LESS_EQUAL")	return D3D11_COMPARISON_LESS_EQUAL;
	else if(val == "GREATER")	return D3D11_COMPARISON_GREATER;
	else if(val == "NOT_EQUAL")	return D3D11_COMPARISON_NOT_EQUAL;
	else if(val == "GREATER_EQUAL")	return D3D11_COMPARISON_GREATER_EQUAL;
	else if(val == "ALWAYS")	return D3D11_COMPARISON_ALWAYS;
	return D3D11_COMPARISON_ALWAYS;
}

bool parse_state(PASS_ELEM elem, const char* &first, const char* last, zix11::ShaderFXDesc::PASS& pass)
{
	int ary_idx = 0;
	skip_type<const char*> skip;
	namestr_type<const char*, std::string> namestr;

	// [ary_idx]
	qi::parse(first, last,
			*skip >> "[" >> *skip >> int_ >> *skip >> "]" >> *skip,
									ary_idx );
	std::string val;
	if( !qi::parse(first, last,
		"=" >> *skip >> *(alnum|char_('_')) >> *skip >> ";" >> *skip,
						val)
	){
		return false;
	}
	const int BD_RT_MAX = _countof(pass.descBD.RenderTarget);

	switch(elem){
	case PASS_CULLMODE:
		if(		val == "FRONT")	pass.descRS.CullMode = D3D11_CULL_FRONT;
		else if(val == "BACK")	pass.descRS.CullMode = D3D11_CULL_BACK;
		else					pass.descRS.CullMode = D3D11_CULL_NONE;
		break;
	case PASS_BLEND_ENABLE:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].BlendEnable = conv_bool(val);
		break;
	case PASS_SRC_BLEND:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].SrcBlend = conv_blend(val);
		break;
	case PASS_DEST_BLEND:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].DestBlend = conv_blend(val);
		break;
	case PASS_BLEND_OP:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].BlendOp = conv_blendop(val);
		break;
	case PASS_SRC_BLEND_ALPHA:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].SrcBlendAlpha = conv_blend(val);
		break;
	case PASS_DEST_BLEND_ALPHA:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].DestBlendAlpha = conv_blend(val);
		break;
	case PASS_BLEND_OP_ALPHA:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].BlendOpAlpha = conv_blendop(val);
		break;
	case PASS_RT_WRITE_MASK:
		if(ary_idx>=BD_RT_MAX)break;
		pass.descBD.RenderTarget[ary_idx].RenderTargetWriteMask = conv_write_mask(val);
		break;
	case PASS_DEPTH_ENABLE:
		pass.descDS.DepthEnable = conv_bool(val);
		break;
	case PASS_DEPTH_FUNC:
		pass.descDS.DepthFunc = conv_func(val);
		break;
	}

		
	return true;
}

}// ns
namespace zix11{

ShaderFXDesc::PASS::PASS()
{
	descRS = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
	descBD = CD3D11_BLEND_DESC(CD3D11_DEFAULT());
	descDS = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
}
ShaderFXDesc::PASS::PASS(const std::string& name) : strName(name)
{
	//�Ϗ��R���X�g���N�^���Ή� VS2012Update1
	// �v�������[�X��CTP�Ȃ�Ή����Ă���炵���������Ή���҂�
	descRS = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
	descBD = CD3D11_BLEND_DESC(CD3D11_DEFAULT());
	descDS = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
}

//-----------------------------------------------------------------
bool CreateShaderFxDesc(const char* data, size_t size, ShaderFXDesc& desc)
{
	const char* first = data;
	const char* last = data + size;
	namestr_type<const char*, std::string> namestr;
	skip_type<const char*> skip;
	comment_type<const char*> comment;
	qi::symbols<char, PASS_ELEM> sym_pass;
	sym_pass.add
		("VertexShader", PASS_VERTEXSHADER)
		("PixelShader", PASS_PIXELSHADER)
		("CullMode", PASS_CULLMODE)
		("BlendEnable", PASS_BLEND_ENABLE)
		("SrcBlend", PASS_SRC_BLEND)
		("DestBlend", PASS_DEST_BLEND)
		("BlendOp", PASS_BLEND_OP)
		("SrcBlendAlpha", PASS_SRC_BLEND_ALPHA)
		("DestBlendAlpha", PASS_DEST_BLEND_ALPHA)
		("BlendOpAlpha", PASS_BLEND_OP_ALPHA)
		("RenderTargetWriteMask", PASS_RT_WRITE_MASK)
		("DepthEnable", PASS_DEPTH_ENABLE)
		("DepthFunc", PASS_DEPTH_FUNC)
		;

	// #ifdef ZGSFX�܂ŃX�L�b�v
	if( !qi::parse(first, last,
			*((*(char_-'#')>>'#') - ("ifdef">>+space>>"ZGFX") ) >> "ifdef" >> +space >>"ZGFX" >> *skip
		)
	){
		return false;
	}
	while(first != last){
		// technique �e�N�j�b�N�� {
		std::string tech_name;
		if( !qi::parse(first, last,
				"technique" >> +skip >> namestr >> *skip >> "{" >> *skip
								, tech_name )
		){
			//zgsfx��`�I��
			if( !qi::parse(first,last, "#endif") ){
				return false;
			}
			break;
		}
		
		desc.aTech.push_back( ShaderFXDesc::TECHNIQUE(tech_name) );
		ShaderFXDesc::TECHNIQUE& desc_tech = desc.aTech.back();

		while(first != last){
			// pass �p�X�� {
			std::string pass_name;
			if( !qi::parse(first, last,
					"pass" >> +skip >> namestr >> *skip >> "{" >> *skip
									,pass_name )
			){
				// technique { "}"
				if( !qi::parse(first, last, "}" >> *skip) ){
					return false;
				}
				break;
			}
			desc_tech.aPass.push_back( ShaderFXDesc::PASS(pass_name) );
			ShaderFXDesc::PASS& desc_pass = desc_tech.aPass.back();

			while(first != last){
				PASS_ELEM pass_elem;
				std::string target, entry;
				if( qi::parse(first, last, sym_pass >> *skip, pass_elem) ){
					switch(pass_elem){
					// V/PShader = compile target entryfunc();
					case PASS_VERTEXSHADER:
					case PASS_PIXELSHADER:
						// = compile  
						if( !qi::parse(first, last, "=" >> *skip >> "compile" >> +skip) ){
							return false;
						}
						// target entryfunc();
						if( !qi::parse(first, last,
								namestr >> +skip >> namestr >> "(" >> *skip >> ")" >> *skip >> ";" >> *skip,
								target, entry
							)
						){
							return false;
						}
						if( pass_elem == PASS_VERTEXSHADER){
							desc_pass.shdVertex.strEntry = entry;
							desc_pass.shdVertex.strTarget = target;
						}else{
							desc_pass.shdPixel.strEntry = entry;
							desc_pass.shdPixel.strTarget = target;
						}
						break;
					default:
						if( !parse_state(pass_elem, first,last, desc_pass) ){
							return false;
						}
						break;
					}
				}else{
					// pass { "}"
					if( !qi::parse(first, last, "}" >> *skip) ){
						return false;
					}
					break;
				}
			}//while pass�v�f

		}//while pass
	}//while technique
	return true;
}

}//zix11