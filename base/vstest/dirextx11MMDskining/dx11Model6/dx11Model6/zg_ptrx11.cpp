#include "stdafx.h"
#include "zg_ptrx11.h"

#include <d3d11.h>

void intrusive_ptr_add_ref(IUnknown* p)
{
	p->AddRef();
}

void intrusive_ptr_release(IUnknown* p)
{
	p->Release();
}

namespace zix11{



}//zix11
