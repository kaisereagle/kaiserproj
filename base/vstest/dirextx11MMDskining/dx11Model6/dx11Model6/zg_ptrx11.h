#pragma once

// DirectX11スマートポインタ
#include <d3d11.h>
#include <boost/utility.hpp>
#include <boost/intrusive_ptr.hpp>

struct IUnknown;
void intrusive_ptr_add_ref(IUnknown* p);
void intrusive_ptr_release(IUnknown* p);

namespace zix11 {//directX11実装

//------------------------------------
// DX11(COM)オブジェクトポインタ
template <typename TYPE>
struct DXPtr
{
	typedef boost::intrusive_ptr<typename TYPE> Type;
};

//C++11 未対応 エイリアステンプレート
//template <typename TYPE>
//using DXPtr = boost::intrusive_ptr<TYPE>;

//------------------------------------
template <typename TYPE>
class GSetDXPtr : boost::noncopyable
{
public:
	GSetDXPtr(TYPE& ptr) : pObj(nullptr), rPtr(ptr){}

	~GSetDXPtr()
	{
		rPtr.swap(TYPE(pObj,false));
	}
	
	typename TYPE::element_type** operator()(){return &pObj;}

private:
	typename TYPE::element_type* pObj;
	TYPE& rPtr;
};

//---------------------------------
#define GSET_DXPTR(_dxptr) GSetDXPtr<std::decay<decltype(_dxptr)>::type>(_dxptr)()
//GSetDXPtr<decltype(_dxptr)>に配列[]を指定すると参照型になりエラー
//std::decayで変換

//-----------------------------------
// DX11オブジェクトポインタ短縮形
typedef DXPtr<ID3D11Device>::Type			IDevicePtr;
typedef DXPtr<ID3D11DeviceContext>::Type	IDevCtxPtr;
typedef DXPtr<IDXGISwapChain>::Type			ISChainPtr;

typedef DXPtr<ID3D11Buffer>::Type			IBuffPtr;
typedef DXPtr<ID3D11Texture2D>::Type		ITex2DPtr;

typedef DXPtr<ID3D11RasterizerState>::Type		IRsStatePtr;	
typedef DXPtr<ID3D11BlendState>::Type			IBdStatePtr;	
typedef DXPtr<ID3D11DepthStencilState>::Type	IDsStatePtr;	

typedef DXPtr<ID3D11VertexShader>::Type		IVShaderPtr;
typedef DXPtr<ID3D11PixelShader>::Type		IPShaderPtr;
typedef DXPtr<ID3D11InputLayout>::Type		IILayoutPtr;
typedef DXPtr<ID3D11SamplerState>::Type		ISpStatePtr;

typedef DXPtr<ID3D11ShaderResourceView>::Type	ISRViewPtr;
typedef DXPtr<ID3D11RenderTargetView>::Type		IRTViewPtr;
typedef DXPtr<ID3D11DepthStencilView>::Type		IDSViewPtr;

typedef DXPtr<ID3DBlob>::Type	IBlobPtr;

}//ix11
