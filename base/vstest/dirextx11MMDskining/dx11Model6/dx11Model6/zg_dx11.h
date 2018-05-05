#pragma once

// DirectX11初期化
// hwnd : 表示するウインドウのハンドル
HRESULT InitDX11(HWND hwnd);

// DirectX11終了
void ExitDX11();

// レンダリング
void RenderDX11();

