#include "zg_types.h"
#include <d3d11.h>

#include <vector>

namespace zix11{

// technique‚âpass‚Ì’è‹`
class ShaderFXDesc
{
public:
	struct SHADER{
		std::string strTarget;
		std::string strEntry;
	};
	struct PASS{
		PASS();
		PASS(const std::string& name);
		std::string strName;
		SHADER shdVertex;
		SHADER shdPixel;
		D3D11_RASTERIZER_DESC descRS;
		D3D11_BLEND_DESC descBD;
		D3D11_DEPTH_STENCIL_DESC descDS;

	};
	struct TECHNIQUE{
		TECHNIQUE(){}
		TECHNIQUE(const std::string& name) : strName(name){}
		std::string strName;
		std::vector<PASS> aPass;
	};

	std::vector<TECHNIQUE> aTech;
private:
};

// #ifdef ZGSFX #endif‚ÅˆÍ‚Ü‚ê‚½•”•ª‚ğ‰ğÍ‚µ‚ÄShaderFXDescì¬
bool CreateShaderFxDesc(const char* data, size_t size, ShaderFXDesc& desc);

}//zix11
